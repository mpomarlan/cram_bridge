;;; Copyright (c) 2013, Georg Bartels <georg.bartels@cs.uni-bremen.de>
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

(in-package :cram-fccl)

(defgeneric to-msg (data))

(defmethod to-msg ((feature geometric-feature))
  (roslisp:make-msg
   "fccl_msgs/feature"
   name (to-msg (name feature))
   reference (to-msg (reference-id feature))
   ; TODO(Georg): refactor message to use primitive messages
   type (roslisp:make-msg
         "std_msgs/Uint8"
         data (ecase (feature-type feature)
                (line
                 (get-feature-type-msg-symbol-code :line))
                (plane
                 (get-feature-type-msg-symbol-code :plane))
                (point
                 (get-feature-type-msg-symbol-code :point))))
   position (to-msg (feature-position feature))
   direction (to-msg (feature-direction feature))))

(defmethod to-msg ((constraint geometric-constraint))
  (roslisp:make-msg
   "fccl_msgs/constraint"
   name (to-msg (name constraint))
   reference (to-msg (reference-id constraint))
   function (to-msg (constraint-function constraint))
   tool_feature (to-msg (tool-feature constraint))
   object_feature (to-msg (object-feature constraint))
   ; TODO(Georg): refactor message to use primitive types
   lower_boundary (roslisp:make-msg
                   "std_msgs/Float64"
                   data (lower-boundary constraint))
   upper_boundary (roslisp:make-msg
                   "std_msgs/Float64"
                   data (upper-boundary constraint))))

(defmethod to-msg ((list-of-data list))
  (map 'vector #'identity
       (map 'list #'to-msg list-of-data)))

(defmethod to-msg ((chain kinematic-chain))
  (roslisp:make-msg
   "fccl_msgs/kinematicchain"
   base_frame (to-msg (base-frame-id chain))
   tip_frame (to-msg (tip-frame-id chain))))

(defmethod to-msg ((point cl-transforms:3d-vector))
  (roslisp:make-msg
   "geometry_msgs/vector3"
   x (cl-transforms:x point)
   y (cl-transforms:y point)
   z (cl-transforms:z point)))

(defmethod to-msg ((string-data string))
  (roslisp:make-msg
   "std_msgs/String"
   data string-data))

(defun get-feature-type-msg-symbol-code (type-symbol)
  (roslisp-msg-protocol:symbol-code
   'fccl_msgs-msg:feature
   type-symbol))

(defun get-feature-type-symbol-from-msg-code (type-code)
  (car 
   (find type-code (roslisp-msg-protocol:symbol-codes 'fccl_msgs-msg:feature) :key #'cdr)))

(defgeneric from-msg (data))

(defmethod from-msg ((msg fccl_msgs-msg:SingleArmMotionFeedback))
  (with-fields (constraints) msg
    (from-msg constraints)))

(defmethod from-msg ((vector-of-msgs vector))
  (map 'list #'identity
       (map 'vector #'from-msg vector-of-msgs)))

(defmethod from-msg ((msg fccl_msgs-msg:ConstraintFeedback))
  (with-fields (command output) msg
    (make-geometric-constraint-feedback (from-msg command) (from-msg output))))

(defmethod from-msg ((msg fccl_msgs-msg:ConstraintState))
  (roslisp:with-fields (output_value desired_output weight) msg
    (make-geometric-constraint-state output_value desired_output weight)))

(defmethod from-msg ((msg fccl_msgs-msg:Constraint))
  (with-fields (name reference function tool_feature object_feature
                     lower_boundary upper_boundary) msg
    ;; TODO(Georg): use primitive datatypes in msg
    (make-geometric-constraint
     (from-msg name)
     (from-msg reference)
     (from-msg function)
     (from-msg tool_feature)
     (from-msg object_feature)
     (from-msg lower_boundary)
     (from-msg upper_boundary))))

(defmethod from-msg ((msg fccl_msgs-msg:Feature))
  (with-fields ((name-msg name) (reference-msg reference) (type-msg type)
                (position-msg position) (direction-msg direction)) msg
      (let ((name (from-msg name-msg))
            (reference (from-msg reference-msg))
            (type (from-msg type-msg))
            (position (from-msg position-msg))
            (direction (from-msg direction-msg)))
        (cond ((eq (get-feature-type-symbol-from-msg-code type) :point)
               (make-point-feature name reference position))
              ((eq (get-feature-type-symbol-from-msg-code type) :line)
               (make-line-feature name reference position direction))
              ((eq (get-feature-type-symbol-from-msg-code type) :plane)
               (make-plane-feature name reference position direction))
              ;; TODO(Georg): throw meaningful error error
              (t nil)))))

(defmethod from-msg ((msg geometry_msgs-msg:Vector3))
  (with-fields (x y z) msg
      (cl-transforms:make-3d-vector x y z)))

(defmethod from-msg ((msg std_msgs-msg:String))
  (with-fields (data) msg
    data))

(defmethod from-msg ((msg std_msgs-msg:Float64))
  (with-fields ((data2 data)) msg 
    data2))

(defmethod from-msg ((msg std_msgs-msg:Uint8))
  (with-fields (data) msg
    data))