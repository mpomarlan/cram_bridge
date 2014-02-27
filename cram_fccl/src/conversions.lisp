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

(in-package :cram-fccl)

(defgeneric to-msg (data))
(defgeneric from-msg (data))

(defmethod to-msg ((motion motion-phase))
  (coerce (mapcar #'to-msg (constraints motion)) 'vector))

(defmethod to-msg ((constraint feature-constraint))
  (with-slots (relation) constraint
  (roslisp:make-msg
   "fccl_msgs/constraint"
   name (to-msg (id constraint))
   reference (to-msg (reference relation))
   function (to-msg (string-downcase (symbol-name (function-type relation))))
   tool_feature (to-msg (tool-feature relation))
   object_feature (to-msg (object-feature relation))
   ; TODO(Georg): refactor message to use primitive types
   lower_boundary (roslisp:make-msg
                   "std_msgs/Float64"
                   data (lower-boundary constraint))
   upper_boundary (roslisp:make-msg
                   "std_msgs/Float64"
                   data (upper-boundary constraint)))))

(defmethod to-msg ((feature geometric-feature))
  (roslisp:make-msg
   "fccl_msgs/feature"
   name (to-msg (id feature))
   reference (to-msg (frame-id feature))
   ; TODO(Georg): refactor message to use primitive messages
   type (roslisp:make-msg
         "std_msgs/Uint8"
         data (lookup-feature-type-msg-symbol-code feature))
   position (to-msg (origin feature))
   direction (to-msg (orientation feature))))

(defmethod to-msg ((chain kinematic-chain))
  (make-msg 
   "fccl_msgs/kinematicchain"
   base_frame (to-msg (base-frame-id chain))
   tip_frame (to-msg (tip-frame-id chain))))

(defmethod to-msg ((point cl-transforms:3d-vector))
  (make-msg
   "geometry_msgs/vector3"
   x (cl-transforms:x point)
   y (cl-transforms:y point)
   z (cl-transforms:z point)))

(defmethod to-msg ((string-data string))
  (make-msg "std_msgs/String" data string-data))

(defmethod to-msg ((list-of-data list))
  (map 'vector #'identity
       (map 'list #'to-msg list-of-data)))

(defun lookup-feature-type-msg-symbol-code (feature)
  (declare (type geometric-feature feature))
  (flet ((feature->feature-type-keyword (feature)
           (ecase (feature-type feature)
             (line :line)
             (plane :plane)
             (point :point)))
         (feature-type-keyword->msg-symbol-code (feature-type-keyword)
           (symbol-code 'fccl_msgs-msg:feature feature-type-keyword)))
    (feature-type-keyword->msg-symbol-code
     (feature->feature-type-keyword feature))))

(defmethod from-msg ((msg fccl_msgs-msg:SingleArmMotionFeedback))
  (with-fields (constraints) msg
    (from-msg constraints)))

(defmethod from-msg ((vector-of-msgs vector))
  (map 'list #'identity
       (map 'vector #'from-msg vector-of-msgs)))

(defmethod from-msg ((msg fccl_msgs-msg:SingleArmMotionFeedback))
  (with-fields (constraints) msg
    (map 'list #'from-msg constraints)))

(defmethod from-msg ((msg fccl_msgs-msg:ConstraintFeedback))
  (with-fields (command output) msg
    (with-fields (output_value desired_output weight) output
      (with-fields (name) command
        (make-feature-constraint-state
         :constraint-id (from-msg name)
         :output output_value
         :ctrl-output desired_output
         :ctrl-weight weight)))))

(defmethod from-msg ((msg fccl_msgs-msg:Constraint))
  (with-fields (name reference function tool_feature object_feature
                     lower_boundary upper_boundary) msg
    ;; TODO(Georg): use primitive datatypes in msg
    (make-feature-constraint
     :id (from-msg name)
     :relation
     (make-feature-relation
      :reference (from-msg reference)
      :function-type (lookup-relation-type-from-msg function)
      :tool-feature (from-msg tool_feature)
      :object-feature (from-msg object_feature))
     :lower-boundary (from-msg lower_boundary)
     :upper-boundary (from-msg upper_boundary))))

(defun lookup-relation-type-from-msg (msg)
  (declare (type std_msgs-msg:String msg))
  (find (from-msg msg) (valid-relation-functions) :key #'symbol-name :test #'string-equal))
  
(defmethod from-msg ((msg fccl_msgs-msg:Feature))
  (with-fields ((name-msg name) (reference-msg reference) (type-msg type)
                (position-msg position) (direction-msg direction)) msg
    (make-geometric-feature
     :id (from-msg name-msg)
     :frame-id (from-msg reference-msg)
     :feature-type (lookup-feature-type-from-msg type-msg)
     :origin (from-msg position-msg)
     :orientation (from-msg direction-msg))))

(defun lookup-feature-type-from-msg (msg)
  (declare (type std_msgs-msg:Uint8 msg))
  (flet ((type-code->feature-type-keyword (type-code)
           (car (find type-code (symbol-codes 'fccl_msgs-msg:feature) :key #'cdr)))
         (feature-type-keyword->feature-type-symbol (type-keyword)
           (ecase type-keyword
             (:point 'point)
             (:line 'line)
             (:plane 'plane))))
    (feature-type-keyword->feature-type-symbol
     (type-code->feature-type-keyword (from-msg msg)))))

(defmethod from-msg ((msg geometry_msgs-msg:Vector3))
  (with-fields (x y z) msg
      (cl-transforms:make-3d-vector x y z)))

(defmethod from-msg ((msg std_msgs-msg:String))
  (with-fields (data) msg
    data))

(defmethod from-msg ((msg std_msgs-msg:Float64))
  (with-fields (data) msg 
    data))

(defmethod from-msg ((msg std_msgs-msg:Uint8))
  (with-fields (data) msg
    data))