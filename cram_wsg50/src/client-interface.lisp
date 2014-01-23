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
         (open-client (make-service-client open-service-name "wsg_50_common/Move"))
         (close-client (make-service-client close-service-name "wsg_50_common/Move")))
  (make-instance 'wsg50-interface :open-client open-client :close-client close-client)))

(defun open-gripper (interface &key (width *completely-open-width*) (speed *default-speed*))
  "Opens the Schunk WSG50 gripper behind `interface' width a set-point of `width' (in mm),
 and a speed of `speed' (mm/s). Might throw a condition of type 'wsg50-command-error'."
  (declare (type wsg50-interface interface))
  (with-fields (error)
      (call-service (open-client interface) :width width :speed speed)
    (unless (error-code-fine-p error)
      (error 'wsg50-command-error 
             :test "An error occured during opening of gripper."
             :error-code error))))

(defun close-gripper (interface &key (width *completely-closed-width*)
                                  (speed *default-speed*))
  "Closes the Schunk WSG50 gripper behind `interface' width a set-point of `width' (in mm),
 and a speed of `speed' (mm/s). Might throw a condition of type 'wsg50-command-error'."
  (declare (type wsg50-interface interface))
  (with-fields (error)
      (call-service (close-client interface) :width width :speed speed)
    (unless (error-code-fine-p error)
      (error 'wsg50-command-error 
             :text "An error occured during closing of gripper."
             :error-code error))))

(define-condition wsg50-command-error (error)
  ((text :initarg :text :reader text)
   (error-code :initarg :error-code :reader error-code))
  (:documentation "Condition signalling an error commanding Schunk WSG50 gripper."))

;;; INTERNAL AUXILIARY FUNCTIONS

(defun error-code-fine-p (error-code)
  (declare (type number error-code))
  (eql error-code *no-error-occured-code*))