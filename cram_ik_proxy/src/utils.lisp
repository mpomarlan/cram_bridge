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

(in-package :cram-ik-proxy)

(defun get-solver-info (service-namespace)
  "Queries the IK server in namespace `service-namespace' for the root and tip link of the
 kinematic chain, and for the list of joint-names in the chain. Returns all three in this
 order."
  (declare (type string service-namespace))
  (let* ((client-name (concatenate 'string service-namespace "/get_ik_solver_info")))
    (let ((response (roslisp:call-service client-name "iai_kinematics_msgs/GetKinematicSolverInfo")))
      (roslisp:with-fields (kinematic_solver_info) response
        (roslisp:with-fields (link_names joint_names) kinematic_solver_info
          (let ((tip-link (elt link_names 0))
                (root-link (elt link_names (- (length link_names) 1)))
                (joint-names (coerce joint_names 'list)))
            (values root-link tip-link joint-names)))))))

(defun make-ik-request (interface goal-pose seed-state)
  "Creates and returns an instance of type 'iai_kinematics_msgs/PositionIKRequest' filled
 with the contents of `interface', `goal-pose' and `seed-state'."
  (declare (type ik-proxy-interface interface)
           (type cl-tf:pose-stamped goal-pose)
           (type list seed-state))
  (roslisp:make-msg
   "iai_kinematics_msgs/PositionIKRequest"
   :ik_link_name (ik-tip-link interface)
   :pose_stamped (tf:pose-stamped->msg goal-pose)
   :ik_seed_state (make-seed-state-msg interface seed-state)))

(defun make-seed-state-msg (interface seed-state)
  "Creates and returns an instance of type 'iai_kinematics_msgs/RobotState' filled with the
 content of `interface' and `seed-state'."
  (declare (type list seed-state)
           (type ik-proxy-interface interface))
  (roslisp:make-msg 
   "iai_kinematics_msgs/RobotState"
   :joint_state (roslisp:make-msg 
                 "sensor_msgs/JointState"
                 :name (coerce (joint-names interface) 'vector)
                 :position (coerce seed-state 'vector)
                 :velocity (make-array (length seed-state) :initial-element 0)
                 :effort (make-array (length seed-state) :initial-element 0))))