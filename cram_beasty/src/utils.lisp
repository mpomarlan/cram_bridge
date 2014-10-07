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

(in-package :cram-beasty)

;;;
;;; ROS ACTION UTILS
;;;

(defun action-succeeded-p (client)
  (eql (state client) :SUCCEEDED))

(defmacro make-beasty-goal (goal-description)
  `(apply #'roslisp::make-message-fn 
          *beasty-goal-type* ,goal-description))

(defun extract-cmd-id (action-result command-code)
  (with-fields ((cmd-id (cmd_id com state))) action-result 
    (elt cmd-id command-code)))

(defun get-beasty-command-code (command-symbol)
  "Returns the Beasty command-code defined in dlr_msgs/RCUGoal which corresponds to
   `command-symbol'."
  (declare (type symbol command-symbol))
  (roslisp-msg-protocol:symbol-code 'dlr_msgs-msg:rcugoal command-symbol))

;; (defun get-strongest-collision (state)
;;   "Iterates over the vector of joint-collisions in beasty-state `state' and returns the
;;  symbol of the strongest type of collision detected. If nothing detected, :NO-CONTACT will
;;  be returned."
;;   (declare (type beasty-state state))
;;   (let ((collisions 
;;           (if (slot-boundp state 'joint-collisions) (joint-collisions state) nil)))
;;     (labels ((collision-symbol->number (collision)
;;                (case collision
;;                  (:NO-CONTACT 0)
;;                  (:CONTACT 1)
;;                  (:LIGHT-COLLISION 2)
;;                  (:STRONG-COLLISION 3)
;;                  (:SEVERE-COLLISION 4)))
;;              (pick-stronger-collision (collision1 collision2)
;;                (if (> (collision-symbol->number collision1)
;;                       (collision-symbol->number collision2))
;;                    collision1
;;                    collision2)))
;;       (reduce #'pick-stronger-collision collisions
;;               :key #'collision-type :initial-value :NO-CONTACT))))