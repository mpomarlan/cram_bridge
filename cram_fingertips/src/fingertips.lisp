;;; Copyright (c) 2014, Jan Winkler <winkler@cs.uni-bremen.de>
;;; All rights reserved.
;;; 
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;; 
;;;     * Redistributions of source code must retain the above copyright
;;;       notice, this list of conditions and the following disclaimer.
;;;     * Redistributions in binary form must reproduce the above copyright
;;;       notice, this list of conditions and the following disclaimer in the
;;;       documentation and/or other materials provided with the distribution.
;;;     * Neither the name of the Universitaet Bremen nor the names of its contributors 
;;;       may be used to endorse or promote products derived from this software 
;;;       without specific prior written permission.
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

(in-package :cram-fingertips)

(defvar *fingertip-sensor-topic-left* "/pressure/l_gripper_motor")
(defparameter *fingertip-subscriber-left* nil)
(defvar *fingertip-sensor-values-fluent-left* (cpl:make-fluent))

(defvar *fingertip-sensor-topic-right* "/pressure/r_gripper_motor")
(defparameter *fingertip-subscriber-right* nil)
(defvar *fingertip-sensor-values-fluent-right* (cpl:make-fluent))

(defun fingertip-subscriber-left-callback (msg)
  (with-fields (l_finger_tip r_finger_tip) msg
    (cpl:setf (cpl:value *fingertip-sensor-values-fluent-left*)
              (list l_finger_tip r_finger_tip))))

(defun fingertip-subscriber-right-callback (msg)
  (with-fields (l_finger_tip r_finger_tip) msg
    (cpl:setf (cpl:value *fingertip-sensor-values-fluent-right*)
              (list l_finger_tip r_finger_tip))))

(defun init-fingertips ()
  (setf *fingertip-subscriber-left*
        (subscribe *fingertip-sensor-topic-left*
                   "pr2_msgs/PressureState"
                   #'fingertip-subscriber-left-callback))
  (setf *fingertip-subscriber-right*
        (subscribe *fingertip-sensor-topic-right*
                   "pr2_msgs/PressureState"
                   #'fingertip-subscriber-right-callback)))

(roslisp-utilities:register-ros-init-function init-fingertips)

(cpl:define-policy pressure-changed (gripper finger slot threshold)
  (:check (let ((prs (pressure gripper finger)))
            (when prs
              (>= (elt prs slot) threshold)))))

;; (defun wait-for-pressure (gripper finger slot threshold)
;;   (block pressure-checker
;;     (with-failure-handling
;;         ((policy-check-condition-met (f)
;;            (declare (ignore f))
;;            (return-from pressure-checker)))
;;       (cpl:with-policy pressure-changed (gripper finger slot threshold)
;;         (loop do (sleep 1))))))

(defun pressure (gripper finger)
  (ecase gripper
    (:left
     (ecase finger
       (:left (nth 0 (cpl:value *fingertip-sensor-values-fluent-left*)))
       (:right (nth 1 (cpl:value *fingertip-sensor-values-fluent-left*)))))
    (:right
     (ecase finger
       (:left (nth 0 (cpl:value *fingertip-sensor-values-fluent-right*)))
       (:right (nth 1 (cpl:value *fingertip-sensor-values-fluent-right*)))))))
