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

(in-package :cram-beasty)

;;;
;;; TODO(Georg): Move this to data-structures?
;;;

(defclass beasty-handle ()
  ((client :initarg :client :reader client :type actionlib-lisp::action-client
           :documentation "For internal use. ROS action client to talk to Beasty.")
   (lock :initform (make-mutex :name (string (gensym "BEASTY-LOCK-")))
         :accessor lock :type mutex
         :documentation "For internal use. Mutex to guard Beasty-handle.")
   (session-id :initarg :session-id :accessor session-id :type number
               :documentation "For internal use. ID of current communication session.")
   (cmd-id :initarg :cmd-id :accessor cmd-id :type number
           :documentation "For internal use. cmd-id to be used in the next goal."))
  (:documentation "Handle to talk with a Beasty controller for the LWR."))

;;;
;;; PARAMETERS
;;;



;;;
;;; UTILS
;;;

;;;
;;; CL-TRANSFORMS
;;;

(defun transpose-2d-matrix (matrix)
  (declare (type array matrix))
  (assert (= (array-rank matrix) 2))
  (destructuring-bind (rows columns) (array-dimensions matrix)
    (make-array 
     `(,columns ,rows)
     :initial-contents
     (loop for column from 0 below columns collecting
       (loop for row from 0 below rows collecting
         (aref matrix row column))))))

;;;
;;; ACTUAL INTERFACE
;;;

(defun make-beasty-handle (action-name user-id user-pwd
                           &key (exec-timeout *exec-timeout*) 
                             (preempt-timeout *preemt-timeout*)
                             (server-timeout *server-timeout*))
  (let* ((client (make-simple-action-client
                  action-name *beasty-action-type*))
         (command-code (get-beasty-command-code :LOGIN))
         (goal-description 
           `(:command ,command-code
            (:command :com :parameters) ,command-code
            (:user_id :com :parameters) ,user-id
            (:user_pwd :com :parameters) ,user-pwd)))
    (wait-for-server client server-timeout)
    (send-goal-and-wait 
     client (make-beasty-goal-msg goal-description) 
     exec-timeout preempt-timeout)
    (if (action-succeeded-p client)
        (make-instance
         'beasty-handle
         :client client
         :session-id (msg-slot-value (result client) :session_id)
         :cmd-id (extract-cmd-id (result client) command-code))
        (error "Login to Beasty '~S' with ID '~S' and PWD '~S' failed."
               action-name user-id user-pwd))))

(defun move-beasty-and-wait (handle goal-description
                             &key (exec-timeout *exec-timeout*) 
                               (preempt-timeout *preemt-timeout*))
  (with-recursive-lock ((lock handle))
    (send-goal-and-wait
     (client handle) 
     (goal-description-to-msg (add-com-description goal-description handle))
     exec-timeout 
     preempt-timeout)
    (update-cmd-id handle goal-description)))

;;;
;;; INTERNALS OF INTERFACE
;;;

(defun add-com-description (goal-description handle)
  (with-slots (session-id cmd-id) handle
    (setf (getf goal-description :cmd-id) cmd-id)
    (setf (getf goal-description :session-id) session-id))
  goal-description)
 
(defun update-cmd-id (handle goal-description)
  (assert (actionlib-lisp:terminal-state (client handle)))
  (with-recursive-lock ((lock handle))
    (setf (cmd-id handle)
          (extract-cmd-id 
           (result handle) (infer-command-code goal-description)))))

(defun infer-command-code (goal-description)
  (cond
    ((joint-goal-description-p goal-description) 1)
    (t (error "Could not infer command code for ~a~%" goal-description))))

;;;
;;; CONVERSIONS
;;;

(defun goal-description-to-msg (goal-description)
  (unless (goal-description-valid-p goal-description)
    (error "Asked to translate invalid Beasty goal description ~a.~%"
           goal-description))
  (make-beasty-goal-msg
   (rosify-goal-description
    (append-sane-defaults
     (vectorify-goal-description 
      (plist-hash-table-recursively goal-description))))))

(defun goal-description-valid-p (goal-description)
  (declare (ignore goal-description))
  ;;TODO(Georg): implement me
  t)

(defun vectorify-goal-description (goal-description)
  (cond
    ((joint-goal-description-p goal-description)
     (vectorify-joint-goal goal-description))
    ;;TODO: add cartesian case
    (t (error "Translation of beasty goal not yet supported."))))

(defun joint-goal-description-p (goal-description)
  (eql (gethash :command-type goal-description) :joint-impedance))

(defun vectorify-joint-goal (goal-description)
  ;;TODO(Georg): refactor using `add-assocs' and `merge-hash-tables'
  (let ((goal (make-array 7))
        (max-vel (make-array 7))
        (max-acc (make-array 7))
        (stiff (make-array 7))
        (damping (make-array 7))
        (result (alexandria:copy-hash-table goal-description)))
    (mapcar (lambda (key)
              (let ((joint-descr (gethash key goal-description))
                    (joint-index (alexandria:assoc-value *joint-index-map* key)))
                (setf (elt goal joint-index) 
                      (gethash :goal-pos joint-descr))
                (setf (elt max-vel joint-index) 
                      (gethash :max-vel joint-descr))
                (setf (elt max-acc joint-index) 
                      (gethash :max-acc joint-descr))
                (setf (elt stiff joint-index) 
                      (gethash :stiffness joint-descr))
                (setf (elt damping joint-index) 
                      (gethash :damping joint-descr))))
            *joint-symbols*)
    (setf (gethash :joint-goal result) goal)
    (setf (gethash :joint-max-vel result) max-vel)
    (setf (gethash :joint-max-acc result) max-acc)
    (setf (gethash :joint-stiffness result) stiff)
    (setf (gethash :joint-damping result) damping)
    (apply #'remove-keys! result *joint-symbols*)))

(defun append-sane-defaults (goal-description)
  (setf (gethash :motor-power goal-description) 
        (make-array 7 :initial-element 1))
  (setf (gethash :o_t_f goal-description)
        (cl-transforms:make-identity-transform))
  (setf (gethash :o_t_via goal-description)
        (cl-transforms:make-identity-transform))
  (setf (gethash :w_t_op goal-description)
        (cl-transforms:make-identity-transform))
  (setf (gethash :ee_t_k goal-description)
        (cl-transforms:make-identity-transform))
  (setf (gethash :ref_t_k goal-description)
        (cl-transforms:make-identity-transform))
  goal-description)

(defun rosify-goal-description (goal-description)
  (flet ((rosify-entry (key value)
           (case key
             (:command-type
              (case value
                ;; TODO(Georg): replace those numbers with a lookup to symbol-codes
                (:joint-impedance '((:command 1)
                                    ((:command :com :parameters) 1)
                                    ((:mode :controller :parameters) 4)
                                    ((:mode :interpolator :parameters) 5)))
                (otherwise (warn "Asked to convert command-type '~a' of goal description." value))))
             (:simulated-robot `(((:mode :robot :parameters) ,value)))
             (:motor-power `(((:power :robot :parameters) ,value)))
             (:session-id `(((:session_id :com :parameters) ,value)))
             (:cmd-id `(((:cmd_id :com :parameters) ,value)))
             (:joint-goal `(((:q_f :interpolator :parameters) ,value)))
             (:joint-max-vel `(((:dq_max :interpolator :parameters) ,value)))
             (:joint-max-acc `(((:ddq_max :interpolator :parameters) ,value)))
             (:o_t_f `(((:O_T_f :interpolator :parameters)
                        ,(transform-to-beasty-msg value))))
             (:o_t_via `(((:O_T_via :interpolator :parameters)
                          ,(transform-to-beasty-msg value))))
             (:joint-stiffness `(((:K_theta :controller :parameters) ,value)))
             (:joint-damping `(((:D_theta :controller :parameters) ,value)))
             (:ee-transform `(((:TCP_T_EE :settings :parameters) 
                               ,(transform-to-beasty-msg value))))
             (:base-transform `(((:W_T_O :settings :parameters) 
                                 ,(transform-to-beasty-msg value))))
             (:base-acceleration `(((:ddX_O :settings :parameters)
                                    ,(wrench-to-msg value))))
             (:tool-com `(((:ml_com :settings :parameters) ,(3d-point-to-msg value))))
             (:tool-mass `(((:ml :settings :parameters) ,value)))
             (:w_t_op `(((:W_T_OP :settings :parameters)
                         ,(transform-to-beasty-msg value))))
             (:ee_t_k `(((:EE_T_K :settings :parameters)
                         ,(transform-to-beasty-msg value))))
             (:ref_t_k `(((:Ref_T_K :settings :parameters)
                         ,(transform-to-beasty-msg value))))
             (otherwise (warn "Asked to convert unknown entry '~a' of goal description." 
                              (list key value))))))
    (reduce #'append 
            (reduce #'append
                    (loop for k being the hash-key in goal-description using (hash-value v)
                          collect (rosify-entry k v))))))

(defun transform-to-beasty-msg (transform)
  (let* ((array4x4 (transpose-2d-matrix (cl-transforms:transform->matrix transform))))
    (make-array (array-total-size array4x4)
                :element-type (array-element-type array4x4)
                :displaced-to array4x4)))

(defun pose-to-beasty-msg (pose)
  (transform-to-beasty-msg
   (cl-transforms:pose->transform pose)))

(defun twist-to-msg (twist)
  (concatenate' 
   vector 
   (3d-point-to-msg (cl-transforms:translation twist))
   (3d-point-to-msg (cl-transforms:rotation twist)))) 

(defun wrench-to-msg (wrench)
  (concatenate' 
   vector 
   (3d-point-to-msg (cl-transforms:translation wrench))
   (3d-point-to-msg (cl-transforms:rotation wrench))))

(defun 3d-point-to-msg (point)
  (vector (cl-transforms:x point) (cl-transforms:y point) (cl-transforms:z point)))

;;;
;;; TEST CASES/EXAMPLES
;;;

(defparameter *sample-joint-goal-description*
  `(:command-type :joint-impedance
     :simulated-robot t
     :joint0 (:stiffness 100
              :damping 0.7
              :max-vel 0.10
              :max-acc 0.40
              :goal-pos 0.0)
     :joint1 (:stiffness 101
              :damping 0.7
              :max-vel 0.11
              :max-acc 0.41
              :goal-pos 0.1)
     :joint2 (:stiffness 102
              :damping 0.7
              :max-vel 0.12
              :max-acc 0.42
              :goal-pos 0.2)
     :joint3 (:stiffness 103
              :damping 0.7
              :max-vel 0.13
              :max-acc 0.43
              :goal-pos 0.3)
     :joint4 (:stiffness 104
              :damping 0.7
              :max-vel 0.14
              :max-acc 0.44 :goal-pos 0.4)
     :joint5 (:stiffness 105
              :damping 0.7
              :max-vel 0.15
              :max-acc 0.45
              :goal-pos 0.5)
     :joint6 (:stiffness 106
              :damping 0.7
              :max-vel 0.16
              :max-acc 0.6
              :goal-pos 0.6)
     :ee-transform ,(cl-transforms:make-identity-transform)
     :base-transform ,(cl-transforms:make-identity-transform)
     :base-acceleration ,(cl-transforms:make-identity-wrench)
     :tool-mass 0.0
     :tool-com ,(cl-transforms:make-identity-vector)))

(defparameter *sample-handle*
  (make-instance 
   'beasty-handle :session-id 123 :cmd-id 456))

(defparameter *extended-sample-joint-goal-description*
  (add-com-description *sample-joint-goal-description* *sample-handle*))