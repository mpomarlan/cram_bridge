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

(defgeneric toMsg (data))

(defmethod toMsg ((feature geometric-feature))
  (roslisp:make-msg
   "fccl_msgs/feature"
   name (name feature)
   reference (reference-id feature)
   type (ecase (feature-type feature)
          (line
           (get-feature-type-msg-symbol-code :line))
          (plane
           (get-feature-type-msg-symbol-code :plane))
          (point
           (get-feature-type-msg-symbol-code :point)))
   position (3d-vector->vector3-msg
             (feature-position feature))
   direction (3d-vector->vector3-msg
              (feature-direction feature))))

;; (defun feature-constraint->single-config-msg (constraint)
;;   (declare (type feature-constraint constraint))
;;   (roslisp:make-msg
;;    "constraint_msgs/constraint"
;;    name (name constraint)
;;    function (feature-function constraint)
;;    tool_feature (feature->msg (tool-feature constraint))
;;    world_feature (feature->msg (world-feature constraint))))

;; (defun feature-constraints->config-msg (constraints controller-id)
;;   (declare (type list constraints)
;;            (type string controller-id))
;;   (let ((constraint-msg-vector
;;            (map 'vector #'identity
;;                 (map 'list #'feature-constraint->single-config-msg constraints))))
;;     (roslisp:make-msg
;;      "constraint_msgs/constraintconfig"
;;      :controller_id controller-id
;;      :movement_id (sxhash constraints)
;;      :constraints constraint-msg-vector)))

;; (defun feature-constraints->command-msg (constraints controller-id)
;;   (declare (type list constraints)
;;            (type string controller-id))
;;   (let ((min_vels
;;           (map 'list #'minimum-velocity constraints))
;;         (max_vels
;;           (map 'list #'maximum-velocity constraints))
;;         (weights
;;           (map 'list #'weight constraints))
;;         (lower
;;           (map 'list #'lower-boundary constraints))
;;         (upper
;;           (map 'list #'upper-boundary constraints)))
;;     (roslisp:make-msg
;;      "constraint_msgs/constraintcommand"
;;      controller_id controller-id
;;      movement_id (sxhash constraints)
;;      pos_lo (map 'vector #'identity
;;                  lower)
;;      pos_hi (map 'vector #'identity
;;                  upper)
;;      weight (map 'vector #'identity
;;                  weights)
;;      max_vel (map 'vector #'identity
;;                   max_vels)
;;      min_vel (map 'vector #'identity
;;                   min_vels))))

(defun 3d-vector->vector3-msg (point)
  (declare (type cl-transforms:3d-vector point))
  (roslisp:make-msg
   "geometry_msgs/vector3"
   x (cl-transforms:x point)
   y (cl-transforms:y point)
   z (cl-transforms:z point)))

(defun get-feature-type-msg-symbol-code (type-symbol)
  (roslisp-msg-protocol:symbol-code
   'fccl_msgs-msg:feature
   type-symbol))

;; (defun constraint-state-msg->feature-constraint-state (msg)
;;   (when msg
;;     (roslisp:with-fields (weights movement_id) msg
;;       (cram-feature-constraints:make-constraint-state weights movement_id))))