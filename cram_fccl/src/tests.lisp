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

(defun fccl-controller-finished-p (msg)
  "Checks whether all weight entries in 'msg' are smaller than 1.0. If yes T is return, else nil."
  (when msg
    (roslisp:with-fields (weights) msg
      ;; calculate maximum weight over all constraints
      (let ((max-weight (loop for i from 0 below (length weights)
                              for weight = (elt weights i)
                              maximizing weight into max-weight
                              finally (return max-weight))))
        ;; return T if max-weight is greater then 1.0, else 'when' returns nil.
        (when (> max-weight 1.0) t)))))

(defun fccl-pr2-pouring-test ()
  ;; make sure the machinerie of the fccl-interface is initialized
  (ensure-fccl-initialized)
  ;; get a fccl-interface for the left arm of the robot
  (let ((pr2-left-arm-interface
          (add-fccl-controller-interface
           "/left_arm_feature_controller/constraint_config"
           "/left_arm_feature_controller/constraint_command"
           "/left_arm_feature_controller/constraint_state"
           "pr2_left_arm_feature_controller")))
    ;; setup the movement descriptions
    (let ((bottle-top (cram-feature-constraints:make-plane-feature
                       "bottle-cover-top" "/pancake_bottle" 
                       :position (cl-transforms:make-3d-vector 0 0 0.0825)
                       :normal (cl-transforms:make-3d-vector 0 0 0.03)))
          (bottle-axis (cram-feature-constraints:make-line-feature
                        "bottle-main-axis" "/pancake_bottle"
                        :direction (cl-transforms:make-3d-vector 0 0 0.1)))
          (oven-center (cram-feature-constraints:make-plane-feature
                        "oven-center" "/pancake"
                        :normal (cl-transforms:make-3d-vector 0 0 0.1))))
      (let ((top-distance-constraint
              (cram-feature-constraints:make-distance-constraint
               "distance bottle-top to oven-center"
               bottle-top oven-center 0.03 0.07))
            (top-height-constraint
              (cram-feature-constraints:make-height-constraint
               "height bottom-top over oven-center"
               bottle-top oven-center 0.25 0.3))
            (bottle-upright-constraint
              (cram-feature-constraints:make-perpendicular-constraint
               "bottle upright"
               bottle-axis oven-center 0.95 1.2))
            (bottle-pointing-at-oven-center
              (cram-feature-constraints:make-pointing-at-constraint
               "bottle pointing oven center"
               bottle-axis oven-center -0.1 0.1))
            (bottle-tilting-down
              (cram-feature-constraints:make-perpendicular-constraint
               "bottle tilting down"
               bottle-axis oven-center -0.2 -0.1)))
        (let ((constraints-phase1 (list top-distance-constraint
                                        top-height-constraint
                                        bottle-upright-constraint))
              (constraints-phase2 (list top-distance-constraint
                                        top-height-constraint
                                        bottle-pointing-at-oven-center
                                        bottle-tilting-down)))
          (let ((motion-phases (list constraints-phase1 constraints-phase2)))
            ;; greedily execute the phases one at a time
            (loop for phase in motion-phases 
                  do (progn
                       ;; start motion execution
                       (execute-constraints-motion phase pr2-left-arm-interface)
                       ;; wait for fluent to finish
                       (cram-language:wait-for 
                        (cram-language:fl-funcall 
                         #'fccl-controller-finished-p 
                         (get-constraints-state-fluent pr2-left-arm-interface)))))))))))
      