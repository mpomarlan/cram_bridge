;;; Copyright (c) 2014, Jannik Buckelo <jannikbu@cs.uni-bremen.de>
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

(in-package :pr2-controllers-examples)

(defparameter *joint-names* '(:l_shoulder_pan_joint 
                              :l_shoulder_lift_joint
                              :l_upper_arm_roll_joint
                              :l_elbow_flex_joint
                              :l_forearm_roll_joint
                              :l_wrist_flex_joint
                              :l_wrist_roll_joint))

(defparameter *goal-state1* '(:l_shoulder_pan_joint (:position 0.0)
                              :l_shoulder_lift_joint (:position 0.0)
                              :l_upper_arm_roll_joint (:position 0.0)
                              :l_elbow_flex_joint (:position 0.0)
                              :l_forearm_roll_joint (:position 0.0)
                              :l_wrist_flex_joint (:position 0.0)
                              :l_wrist_roll_joint (:position 0.0)))

(defparameter *goal-state2* '(:l_shoulder_pan_joint (:position 0.7)
                              :l_shoulder_lift_joint (:position 0.9)
                              :l_upper_arm_roll_joint (:position 1.4)
                              :l_elbow_flex_joint (:position 0.0)
                              :l_forearm_roll_joint (:position 0.0)
                              :l_wrist_flex_joint (:position 0.5)
                              :l_wrist_roll_joint (:position 0.6)))

(defparameter *handle* nil)

(defun init-handle ()
  "Creates and returns a handle for the pr2-controllers"
  (setf *handle*
        (cram-pr2-controllers:make-pr2-arm-position-controller-handle 
         "l_arm_controller/joint_trajectory_action" 
         *joint-names*)))

(defun move-left-arm-straight ()
  "Uses the handle to move the arm to the position specified in *goal-state1* in 10 seconds."
  (cram-pr2-controllers:move-arm *handle* *goal-state1* 10))

(defun move-left-arm-side ()
  "Uses the handle to move the arm to the position specified in *goal-state2* in 2 seconds."
  (cram-pr2-controllers:move-arm *handle* *goal-state2* 2))