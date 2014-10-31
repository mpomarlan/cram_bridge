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

(in-package :beasty-command-interface-examples)

(defparameter *right-arm-joint-goal-zero*
  `(:command-type :joint-impedance
     :simulated-robot t
     :joint0 (:stiffness 80
              :damping 0.7
              :max-vel 0.35
              :max-acc 0.7
              :goal-pos 0.0)
     :joint1 (:stiffness 80
              :damping 0.7
              :max-vel 0.35
              :max-acc 0.7
              :goal-pos 0.0)
     :joint2 (:stiffness 80
              :damping 0.7
              :max-vel 0.35
              :max-acc 0.7
              :goal-pos 0.0)
     :joint3 (:stiffness 80
              :damping 0.7
              :max-vel 0.35
              :max-acc 0.7
              :goal-pos 0.0)
     :joint4 (:stiffness 80
              :damping 0.7
              :max-vel 0.35
              :max-acc 0.7
              :goal-pos 0.0)
     :joint5 (:stiffness 80
              :damping 0.7
              :max-vel 0.35
              :max-acc 0.7
              :goal-pos 0.0)
     :joint6 (:stiffness 80
              :damping 0.7
              :max-vel 0.35
              :max-acc 0.7
              :goal-pos 0.0)
     :ee-transform ,(cl-transforms:make-transform
                     (cl-transforms:make-3d-vector 0 0 0.078)
                     (cl-transforms:make-identity-rotation))
     :base-transform ,(cl-transforms:make-identity-transform)
     :base-acceleration ,(cl-transforms:make-wrench
                          (cl-transforms:make-3d-vector -7.358 4.248 4.905)
                          (cl-transforms:make-identity-vector))
     :tool-mass 0.0
     :tool-com ,(cl-transforms:make-identity-vector)))

(defparameter *right-arm-joint-goal-non-zero*
  `(:command-type :joint-impedance
     :simulated-robot t
     :joint0 (:stiffness 80
              :damping 0.7
              :max-vel 0.35
              :max-acc 0.7
              :goal-pos 0.52)
     :joint1 (:stiffness 80
              :damping 0.7
              :max-vel 0.35
              :max-acc 0.7
              :goal-pos -0.78)
     :joint2 (:stiffness 80
              :damping 0.7
              :max-vel 0.35
              :max-acc 0.7
              :goal-pos 0.78)
     :joint3 (:stiffness 80
              :damping 0.7
              :max-vel 0.35
              :max-acc 0.7
              :goal-pos 0.78)
     :joint4 (:stiffness 80
              :damping 0.7
              :max-vel 0.35
              :max-acc 0.7
              :goal-pos 0.0)
     :joint5 (:stiffness 80
              :damping 0.7
              :max-vel 0.35
              :max-acc 0.7
              :goal-pos -0.78)
     :joint6 (:stiffness 80
              :damping 0.7
              :max-vel 0.35
              :max-acc 0.7
              :goal-pos 0.0)
     :ee-transform ,(cl-transforms:make-transform
                     (cl-transforms:make-3d-vector 0 0 0.078)
                     (cl-transforms:make-identity-rotation))
     :base-transform ,(cl-transforms:make-identity-transform)
     :base-acceleration ,(cl-transforms:make-wrench
                          (cl-transforms:make-3d-vector -7.358 4.248 4.905)
                          (cl-transforms:make-identity-vector))
     :tool-mass 0.0
     :tool-com ,(cl-transforms:make-identity-vector)))

(defparameter *right-arm-cartesian-goal*
  `(:command-type :cartesian-impedance
     :simulated-robot t
     :trans-x (:stiffness 500
               :damping 0.7
               :max-vel 0.2
               :max-acc 0.2)
     :trans-y (:stiffness 500
               :damping 0.7
               :max-vel 0.2
               :max-acc 0.2)
     :trans-z (:stiffness 500
               :damping 0.7
               :max-vel 0.2
               :max-acc 0.2)
     :rot-x (:stiffness 50
             :damping 0.7
             :max-vel 0.7
             :max-acc 0.7)
     :rot-y (:stiffness 50
             :damping 0.7
             :max-vel 0.7
             :max-acc 0.7)
     :rot-z (:stiffness 50
             :damping 0.7
             :max-vel 0.7
             :max-acc 0.7)
     :cartesian-goal-pose 
     ,(cl-transforms:make-transform
       (cl-transforms:make-3d-vector 0.359 0.459 0.533)
       (cl-transforms:axis-angle->quaternion
        (cl-transforms:make-3d-vector 0 1 0) PI))
     :ee-transform ,(cl-transforms:make-transform
                     (cl-transforms:make-3d-vector 0 0 0.078)
                     (cl-transforms:make-identity-rotation))
     :base-transform ,(cl-transforms:make-identity-transform)
     :base-acceleration ,(cl-transforms:make-wrench
                          (cl-transforms:make-3d-vector -7.358 4.248 4.905)
                          (cl-transforms:make-identity-vector))
     :tool-mass 0.0
     :tool-com ,(cl-transforms:make-identity-vector)
     :nullspace-stiffness 0.0
     :nullspace-damping 0.0
     :nullspace-dir ,(cl-transforms:make-identity-vector)))

(defun move-right-arm (&optional (sim-p t))
  (let ((right-arm (beasty-command-interface:make-beasty-handle "right_arm" 1 1337)))
    ;; (query-user-for-acknowledge 
    ;;  "Joint impedance: Right arm into zero configuration.")
    ;; (beasty-command-interface:move-beasty-and-wait 
    ;;  right-arm (set-sim-mode *right-arm-joint-goal-zero* sim-p))
    (query-user-for-acknowledge 
     "Joint impedance: Right arm in forward pointing configuration.")
    (beasty-command-interface:move-beasty-and-wait 
     right-arm (set-sim-mode *right-arm-joint-goal-non-zero* sim-p))
    (query-user-for-acknowledge 
     "Cartesian impedance: Right arm in sidewards pointing configuration.")
    (beasty-command-interface:move-beasty-and-wait 
     right-arm (set-sim-mode *right-arm-cartesian-goal* sim-p))))