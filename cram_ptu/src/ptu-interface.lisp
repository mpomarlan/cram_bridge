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

(in-package :cram-ptu)

(define-condition ptu-command-error (error)
  ((text :initarg :text :reader text))
  (:documentation "Condition signalling an error while moving the PTU."))

(defun make-ptu-interface (action-name)
  "Creates and returns an instance of type `ptu-interface' which is a proxy for the PTU
 controller behind the ROS action with the name `action-name'."
  (declare (type string action-name))
  (let ((tf-broadcaster (cl-tf:make-transform-broadcaster))
        (action-client (actionlib:make-action-client action-name "cogman_msgs/PtuAction")))
    (make-instance
     'ptu-interface :tf-broadcaster tf-broadcaster :action-client action-client)))

(defun cleanup-ptu-interface (interface)
  "Cleans up the internal of ptu-interface `interface'."
  (declare (type ptu-interface interface))
  ;; TODO(Georg): cleanup action-client
  (unadvertise (tf-broadcaster interface)))

(defgeneric point-head (interface destination &key &allow-other-keys)
  (:documentation "Points PTU behind `interface' at `destination'."))

(defmethod point-head ((interface ptu-interface) (point cl-transforms:3d-vector)
                       &key frame-id &allow-other-keys)
  "Points PTU behind `interface' at 3d-point `point' which is defined w.r.t. `frame-id'."
  (declare (type string frame-id))
  (with-recursive-lock ((command-lock interface))
    (let* ((transform 
             (cl-transforms:make-transform point (cl-transforms:make-identity-rotation)))
           (child-frame-id (make-pointing-frame-id))
           (stamp (roslisp:ros-time))
           (stamped-transform 
             (cl-tf:transform->stamped-transform frame-id child-frame-id stamp transform)))
      (with-tf-broadcasting (interface stamped-transform)
        (point-head interface child-frame-id)))))

(defmethod point-head ((interface ptu-interface) (frame-id string)
                       &key &allow-other-keys)
  "Points PTU behind `interface' at TF `frame-id'. In case of failure, throws an instance
 of type 'ptu-command-error'."
  (with-recursive-lock ((command-lock interface))
    (multiple-value-bind (result status)
        (actionlib:send-goal-and-wait
         (action-client interface)
         (actionlib:make-action-goal (action-client interface) :tf_frame frame-id :mode 0))
      (unless (equal :succeeded status)
        (with-fields (answer) result
          (error 'ptu-command-error :text answer))))))