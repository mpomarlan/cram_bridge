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

(defmacro make-beasty-goal-msg (goal-description)
  `(apply #'roslisp::make-message-fn 
          *beasty-goal-type* ,goal-description))

;;;
;;; LIST UTILS
;;;

(defun interleave-lists! (l1 l2)
  ;; TODO(Georg): move to utils
  (reduce #'append (mapcar #'list l1 l2)))

(defun interleave-lists (l1 l2)
  ;; TOOD(Georg): move to utils
  (interleave-lists! (copy-list l1) (copy-list l2)))

;;;
;;; HASH-TABLE UTILS
;;;

(defun plist-hash-table-recursively (plist)
  "Returns a hash table containing the keys and values of the property list
 `plist'. Recursively calls itself on  any of the values of `plist' that are
 also property lists. Note: Leaves `plist' untouched."
  (flet ((plist-p (obj)
           (and (listp obj) (evenp (length obj)) (<= 2 (length obj)))))
  (let ((tmp-list nil))
    (alexandria:doplist (key value plist)
      (setf (getf tmp-list key)
            (if (plist-p value)
                (plist-hash-table-recursively value)
                value)))
    (alexandria:plist-hash-table tmp-list))))

(defun hash-table-has-key-p (hash-table key)
  (multiple-value-bind (value present) (gethash key hash-table)
    (declare (ignore value))
    present))

(defun hash-table-has-keys-p (hash-table keys)
  "Predicate to check whether `hash-table' has non-nil values for all `keys'. 
 Leaves both `hash-table' and `keys' untouched."
  (every (lambda (key) (hash-table-has-key-p hash-table key)) keys))

(defun gethash-recursively (hash-table keys)
"Returns the key inside a nested `hash-table' data-structure, by using the sequence
 of `keys' to make the actual lookup."
  (reduce (lambda (table key) (gethash key table)) keys :initial-value hash-table))

(defun add-assocs! (table &rest kvs)
  (when (/= (rem (length kvs) 2) 0)
    (error "Called add-assocs with an odd number of rest-arguments."))
  (if (= (length kvs) 0)
      table
      (destructuring-bind (key value &rest remainder) kvs
        (setf (gethash key table) value)
        (apply #'add-assocs! table remainder))))

(defun add-assocs (table &rest kvs)
  (apply #'add-assocs! (alexandria:copy-hash-table table) kvs))

(defun remove-keys! (hash-table &rest keys)
  "Removes key-value pairs with `keys' from `hash-table'. NOTE: `hash-table'
 will be altered."
  (if (= (length keys) 0)
      hash-table
      (destructuring-bind (key &rest remainder) keys
        (remhash key hash-table)
        (apply #'remove-keys! hash-table remainder))))

(defun remove-keys (hash-table &rest keys)
  "Returns `hash-table' with all associations with `keys' removed.
 NOTE: `hash-table' will not be touched."
  (apply #'remove-keys! (alexandria:copy-hash-table hash-table) keys))

;;;
;;; KEYWORD UTILS
;;;

(defun prefix-keyword (prefix key)
  (declare (type keyword key) (type string prefix))
  (values 
   (alexandria:make-keyword 
    (concatenate 'string (string-upcase prefix) (symbol-name key)))))

;;;
;;; BEASTY UTILS
;;;

(defun goal-description-valid-p (goal-description)
  (declare (ignore goal-description))
  ;;TODO(Georg): implement me
  t)

(defun joint-goal-description-p (goal-description)
  (eql (gethash :command-type goal-description) :joint-impedance))

(defun cartesian-goal-description-p (goal-description)
  (eql (gethash :command-type goal-description) :cartesian-impedance))

(defun get-beasty-command-code (command-symbol)
  "Returns the Beasty command-code defined in dlr_msgs/RCUGoal which corresponds to
   `command-symbol'."
  (declare (type symbol command-symbol))
  (roslisp-msg-protocol:symbol-code 'dlr_msgs-msg:rcugoal command-symbol))

(defun update-cmd-id (handle goal-msg)
  ;; TODO(Georg): fix me into (terminal-state-p ...)
;  (assert (actionlib-lisp:terminal-state (client handle)))
  (with-recursive-lock ((lock handle))
    (setf (cmd-id handle)
          (extract-cmd-id (result (client handle)) goal-msg))))

(defun infer-command-code-from-description (goal-description)
  "Returns the ROS action command code corresponding to the Beasty `goal-description'."
  ;; TODO(Georg): refactor/delete me! I was once used.. ;)
  (cond
    ((joint-goal-description-p goal-description) 1)
    (t (error "Could not infer command code for ~a~%" goal-description))))

(defun extract-cmd-id (action-result action-goal)
  (with-fields ((cmd-id (cmd_id com state))) action-result 
    (elt cmd-id (msg-slot-value action-goal :command))))

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