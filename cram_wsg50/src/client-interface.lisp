;;; Copyright (c) 2014, Georg Bartels <georg.bartels@cs.uni-bremen.de>
;;; All rights reserved.
;;;
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;;
;;; * Redistributions of source code must retain the above copyright
;;; notice, this list of conditions and the following disclaimer.
;;; * Redistributions in binary form must reproduce the above copyright
;;; notice, this list of conditions and the following disclaimer in the
;;; documentation and/or other materials provided with the distribution.
;;; * Neither the name of the Institute for Artificial Intelligence/
;;; Universitaet Bremen nor the names of its contributors may be used to 
;;; endorse or promote products derived from this software without specific 
;;; prior written permission.
;;;
;;; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;;; AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;;; IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
;;; ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
;;; LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
;;; CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
;;; SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
;;; INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
;;; CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
;;; ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;;; POSSIBILITY OF SUCH DAMAGE.

(in-package :cram-wsg50)

;;; EXPORTED INTERFACE

(defun make-wsg50-interface (namespace)
  "Creates and returns an instance of type 'wsg50-interface':"
  (declare (type string namespace))
  (let* ((open-service-name (concatenate 'string namespace "/release"))
         (close-service-name (concatenate 'string namespace "/grasp"))
         (homing-service-name (concatenate 'string namespace "/homing"))
         (status-topic-name (concatenate 'string namespace "/status"))
         (acc-service-name (concatenate 'string namespace "/set_acceleration"))
         (force-service-name (concatenate 'string namespace "/set_force"))
         (open-client (make-service-client open-service-name "wsg_50_common/Move"))
         (close-client (make-service-client close-service-name "wsg_50_common/Move"))
         (homing-client (make-service-client homing-service-name "std_srvs/Empty"))
         (acc-client (make-service-client acc-service-name "wsg_50_common/Conf"))
         (force-client (make-service-client force-service-name "wsg_50_common/Conf")))
    (let ((interface (make-instance 'wsg50-interface 
                                    :open-client open-client 
                                    :close-client close-client
                                    :homing-client homing-client
                                    :acceleration-client acc-client
                                    :force-client force-client)))
      (add-status-subscriber interface status-topic-name)
      interface)))

(defun open-gripper (interface &key (width *completely-open-width*) (speed *default-speed*)
                                 (acceleration *default-acceleration*)
                                 (force *default-force*))
  "Opens the Schunk WSG50 gripper behind `interface' width a set-point of `width' (in mm),
 and a speed of `speed' (mm/s). Might throw a condition of type 'wsg50-command-error'."
  (declare (type wsg50-interface interface))
  (with-recursive-lock ((command-lock interface))
    (ensure-acceleration interface acceleration)
    (ensure-force interface force)
    (with-fields (error)
        (call-service (open-client interface) :width width :speed speed)
      (unless (error-code-fine-p error)
        (error 'wsg50-command-error 
               :test "An error occured during opening of gripper."
               :error-code error)))))

(defun close-gripper (interface &key (width *completely-closed-width*)
                                  (speed *default-speed*) 
                                  (acceleration *default-acceleration*)
                                  (force *default-force*))
  "Closes the Schunk WSG50 gripper behind `interface' width a set-point of `width' (in mm),
 and a speed of `speed' (mm/s). Might throw a condition of type 'wsg50-command-error'."
  (declare (type wsg50-interface interface))
  (with-recursive-lock ((command-lock interface))
    (ensure-acceleration interface acceleration)
    (ensure-force interface force)
    (with-fields (error)
        (call-service (close-client interface) :width width :speed speed)
      (unless (error-code-fine-p error)
        (error 'wsg50-command-error 
               :text "An error occured during closing of gripper."
               :error-code error)))))

(defun home-gripper (interface)
  "Commands Schunk WSG50 gripper behind `interface' to home."
  (declare (type wsg50-interface interface))
  (with-recursive-lock ((command-lock interface))
    (call-service (homing-client interface))))

(define-condition wsg50-command-error (error)
  ((text :initarg :text :reader text)
   (error-code :initarg :error-code :reader error-code))
  (:documentation "Condition signalling an error commanding Schunk WSG50 gripper."))

;;; INTERNAL AUXILIARY FUNCTIONS

(defun error-code-fine-p (error-code)
  "Internal helper predicate to check whether `error-code' returned from servers running
 Schunk WSG50 gripper indicated successful execution. Returns 't' in case of success, and
 'nil' in case of an error."
  (declare (type number error-code))
  (eql error-code *no-error-occured-code*))

(defun add-status-subscriber (interface status-topic-name)
  "Creates a subscription to ROS topic `status-topic-name' and saves the subscriber in the
 'status-subscriber' slot of `interface'. The callback of the subscribtion will update the
 slot 'status' of interface with the content received over `status-topic-name'."
  (declare (type wsg50-interface interface)
           (type string status-topic-name))
  (let ((subscriber 
          (subscribe status-topic-name "wsg_50_common/Status"
                     (lambda (msg)
                       (with-recursive-lock ((status-lock interface))
                         (setf (status interface) (from-msg msg))))
                     :max-queue-length 1)))
    (setf (status-subscriber interface) subscriber)))

(defun from-msg (msg)
  "Creates an instance of type 'wsg50-status', fills it with content from `msg' which is of
 type 'wsg_50_common/Status', and returns the new object."
  (declare (type wsg_50_common-msg:Status msg))
  (with-fields (width acc force) msg
    (make-instance 'wsg50-status :width width :max-acc acc :max-force force)))

(defun ensure-acceleration (interface acceleration)
  "Makes sure that WSG50 gripper behind `interface' uses an acceleration equal to value of
 `acceleration' for opening and closing."
  (declare (type number acceleration)
           (type wsg50-interface))
  (with-recursive-lock ((status-lock interface))
    (unless (= acceleration (max-acc (status interface)))
      (set-acceleration interface acceleration))))

(defun set-acceleration (interface acceleration)
    "Commands WSG50 gripper behind `interface' to use an acceleration equal to value of
 `acceleration' for opening and closing."
  (declare (type number acceleration)
           (type wsg50-interface))
  (with-recursive-lock ((status-lock interface))
    (with-recursive-lock ((command-lock interface))
      (with-fields (error)
          (call-service (acceleration-client interface) :val acceleration)
        (unless (error-code-fine-p error)
          (error 'wsg50-command-error 
                 :text "An error occured during setting of maximum acceleration."
                 :error-code error))))))

(defun ensure-force (interface force)
  "Makes sure that WSG50 gripper behind `interface' uses a force (in N) equal to value of
 `force' for opening and closing."
  (declare (type number force)
           (type wsg50-interface))
  (with-recursive-lock ((status-lock interface))
    (unless (= force (max-force (status interface))) (set-force interface force))))

(defun set-force (interface force)
  "Commands WSG50 gripper behind `interface' to use a force (in N) equal to value of
 `force' for opening and closing."
  (declare (type number force)
           (type wsg50-interface))
  (with-recursive-lock ((status-lock interface))
    (with-recursive-lock ((command-lock interface))
      (with-fields (error)
          (call-service (force-client interface) :val force)
        (unless (error-code-fine-p error)
          (error 'wsg50-command-error 
                 :text "An error occured during setting of maximum force."
                 :error-code error))))))