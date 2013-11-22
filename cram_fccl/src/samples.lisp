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

(defun test-pr2-left-arm-above-waist ()
  (let ((action-interface (make-action-interface))
        (constraints (make-constraint-specification)))
    (execute-fccl-motion action-interface constraints)))

(defun make-action-interface ()
  (make-fccl-action-interface
   (actionlib:make-action-client
    "/l_arm_fccl_controller/command"
    "fccl_msgs/SingleArmMotionAction")
   (make-kinematic-chain "torso_lift_link" "l_gripper_tool_frame")))

(defun make-constraint-specification ()
  (let ((hand-plane (make-plane-feature
                     "left gripper plane"
                     "l_gripper_tool_frame"
                     (cl-transforms:make-3d-vector 0 0 0)
                     (cl-transforms:make-3d-vector 0 0 1)))
        (waist-plane (make-plane-feature
                      "horizontal waist plane"
                      "torso_lift_link"
                      (cl-transforms:make-3d-vector 0 0 0)
                      (cl-transforms:make-3d-vector 0 0 1))))
    (let ((left-hand-above-waist-constraint 
            (make-geometric-constraint
             "left hand above waist constraint" "base_link" "above"
             hand-plane waist-plane 0.0 2.0)))
      (list left-hand-above-waist-constraint))))