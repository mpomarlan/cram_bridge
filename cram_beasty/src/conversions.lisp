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

(defun goal-description-to-msg (goal-description)
  (unless (goal-description-valid-p goal-description)
    (error "Asked to translate invalid Beasty goal description ~a.~%"
           goal-description))
  (make-beasty-goal-msg
   (rosify-goal-description
    (append-sane-defaults
     (vectorify-goal-description 
      (plist-hash-table-recursively goal-description))))))

(defmacro make-beasty-goal-msg (goal-description)
  `(apply #'roslisp::make-message-fn 
          *beasty-goal-type* ,goal-description))

(defun vectorify-goal-description (goal-description)
  (cond
    ((joint-goal-description-p goal-description)
     (vectorify-joint-goal goal-description))
    ;;TODO: add cartesian case
    (t (error "Translation of beasty goal not yet supported."))))

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
  ;; TODO(Georg): refactor using `add-assocs'
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

;; (define-condition beasty-conversion-error (error)
;;   ((text :initarg :text :reader text))
;;   (:documentation "Condition signalling an error in ROS message creation for Beasty controller."))

;; (defgeneric to-msg (data &key &allow-other-keys)
;;   (:documentation "Creates the ROS message corresponding to lisp-data `data'."))

;; (defgeneric from-msg (msg)
;;   (:documentation "Creates the lisp data structure corresponding to ROS message `msg'."))

;; (defgeneric to-vector (data)
;;   (:documentation "Transforms `data' into a corresponding vector representation."))

;; (defmethod to-msg ((robot beasty-robot) &key &allow-other-keys)
;;   "Creates 'dlr_msgs/tcu2rcu_Robot' and 'dlr_msgs/tcu2rcu_Settings' message using the data
;;    stored in `robot'."
;;   (let ((robot-msg
;;           (roslisp:make-msg "dlr_msgs/tcu2rcu_Robot"
;;                             :mode (if (simulation-flag robot) 1 0)
;;                             :power (if (emergency-released-flag robot)
;;                                         (make-array 7 :initial-element 1)
;;                                         (make-array 7 :initial-element 0))))
;;         (settings-msg
;;           (roslisp:make-msg "dlr_msgs/tcu2rcu_Settings"
;;                             :tcp_t_ee (to-vector (ee-transform (tool-configuration robot)))
;;                             :w_t_o (to-vector (base-transform (base-configuration robot)))
;;                             ;; Cart. velocities for Cartesian velocity control are
;;                             ;; interpreted w.r.t world-frame
;;                             :w_t_op (to-vector (cl-transforms:make-identity-transform))
;;                             ;; Cartesian Impedances are interpreted w.r.t to EE frame
;;                             :ee_t_k (to-vector (cl-transforms:make-identity-transform))
;;                             ;; sane value enforced by beasty
;;                             :ref_t_k (to-vector (cl-transforms:make-identity-transform))
;;                             :ml (mass (tool-configuration robot))
;;                             :ml_com (to-vector (com (tool-configuration robot)))
;;                             :ddx_o (base-acceleration (base-configuration robot)))))
;;     (values robot-msg settings-msg)))

;; (defmethod to-msg ((params gravity-control-parameters) &key &allow-other-keys)
;;   "Creates a 'dlr_msgs/tcu2rcu_Controller' and a 'dlr_msgs/tcu2rcu_Interpolator' message
;;    using the data stored in `params' of type 'gravity-control-parameters'."
;;   (let ((controller-msg (roslisp:make-msg "dlr_msgs/tcu2rcu_Controller" :mode 3))
;;         (interpolator-msg (roslisp:make-msg 
;;                            "dlr_msgs/tcu2rcu_Interpolator"
;;                            :mode 5 ; JOINT-SCALING-INTERPOLATION
;;                            :dq_max (max-joint-vel params)
;;                            :ddq_max (max-joint-acc params)
;;                            ;; sane values enforced by server
;;                            :o_t_f (to-vector (cl-transforms:make-identity-transform))
;;                            :o_t_via (to-vector (cl-transforms:make-identity-transform)))))
;;     (values controller-msg interpolator-msg)))

;; (defmethod to-msg ((params joint-impedance-control-parameters) &key &allow-other-keys)
;;   "Creates a 'dlr_msgs/tcu2rcu_Controller' and a 'dlr_msgs/tcu2rcu_Interpolator' message
;;    using the data stored in `params' of type 'joint-impedance-control-parameters'."
;;   (let ((controller-msg (roslisp:make-msg 
;;                          "dlr_msgs/tcu2rcu_Controller" 
;;                          :mode 4 ; JOINT-IMPEDANCE-MODE
;;                          :k_theta (joint-stiffness params)
;;                          :d_theta (joint-damping params)))
;;         (interpolator-msg (roslisp:make-msg 
;;                            "dlr_msgs/tcu2rcu_Interpolator"
;;                            :mode 5 ; JOINT-SCALING-INTERPOLATION
;;                            :dq_max (max-joint-vel params)
;;                            :ddq_max (max-joint-acc params)
;;                            :q_f (joint-goal params)
;;                            ;; sane values enforced by server
;;                            :o_t_f (to-vector (cl-transforms:make-identity-transform))
;;                            :o_t_via (to-vector (cl-transforms:make-identity-transform)))))
;;     (values controller-msg interpolator-msg)))

;; (defmethod to-msg ((params cartesian-impedance-control-parameters) &key &allow-other-keys)
;;   "Creates a 'dlr_msgs/tcu2rcu_Controller' and a 'dlr_msgs/tcu2rcu_Interpolator' message
;;    using the data stored in `params' of type 'cartesian-impedance-control-parameters'."
;;   (let ((controller-msg (roslisp:make-msg 
;;                          "dlr_msgs/tcu2rcu_Controller" 
;;                          :mode 2 ; CARTESIAN-IMPEDANCE-MODE
;;                          :k_x (cart-stiffness params)
;;                          :d_x (cart-damping params)
;;                          :k_ns (nullspace-stiffness params)
;;                          :xi_ns (nullspace-damping params)
;;                          :w_ns_dir (to-vector (nullspace-dir params))
;;                          :enable_ffwd t
;;                          :dk_x (filter-gains params)))
;;         (interpolator-msg (roslisp:make-msg 
;;                            "dlr_msgs/tcu2rcu_Interpolator"
;;                            :mode 4 ; CARTESIAN-RAMP
;;                            :o_t_f (to-vector (goal-pose params))
;;                            :dx_max (max-cart-vel params)
;;                            :ddx_max (max-cart-acc params)
;;                            ; sane values enforced by Beasty
;;                            :o_t_via (to-vector (cl-transforms:make-identity-transform)))))
;;     (values controller-msg interpolator-msg)))

;; ;; TODO(Georg): the next three are identical; find a way to re-use the methods.
;; (defmethod to-msg ((params hard-stop-parameters) &key &allow-other-keys)
;;   "Creates a 'dlr_msgs/tcu2rcu_Controller' and a 'dlr_msgs/tcu2rcu_Interpolator' message
;;    using the data stored in `params' of type 'complete-stop-parameters'."
;;   (let ((controller-msg (roslisp:make-msg 
;;                          "dlr_msgs/tcu2rcu_Controller" 
;;                          ;; sane values enforce by server
;;                          :mode 4)) ; JOINT-IMPEDANCE-MODE
;;         (interpolator-msg (roslisp:make-msg 
;;                            "dlr_msgs/tcu2rcu_Interpolator"
;;                            ;; sane values enforced by server
;;                            :mode 5 ; JOINT-SCALING-INTERPOLATION
;;                            :o_t_f (to-vector (cl-transforms:make-identity-transform))
;;                            :o_t_via (to-vector (cl-transforms:make-identity-transform)))))
;;     (values controller-msg interpolator-msg)))

;; (defmethod to-msg ((params safety-reset) &key &allow-other-keys)
;;   "Creates a 'dlr_msgs/tcu2rcu_Controller' and a 'dlr_msgs/tcu2rcu_Interpolator' message
;;    using the data stored in `params' of type 'safety-reset'."
;;   (let ((controller-msg (roslisp:make-msg 
;;                          "dlr_msgs/tcu2rcu_Controller" 
;;                          ;; sane values enforce by server
;;                          :mode 4)) ; JOINT-IMPEDANCE-MODE
;;         (interpolator-msg (roslisp:make-msg 
;;                            "dlr_msgs/tcu2rcu_Interpolator"
;;                            ;; sane values enforced by server
;;                            :mode 5 ; JOINT-SCALING-INTERPOLATION
;;                            :o_t_f (to-vector (cl-transforms:make-identity-transform))
;;                            :o_t_via (to-vector (cl-transforms:make-identity-transform)))))
;;     (values controller-msg interpolator-msg)))

;; (defmethod to-msg ((params stop-parameters) &key &allow-other-keys)
;;   "Creates a 'dlr_msgs/tcu2rcu_Controller' and a 'dlr_msgs/tcu2rcu_Interpolator' message
;;    using the data stored in `params' of type 'stop-parameters'."
;;   (let ((controller-msg (roslisp:make-msg 
;;                          "dlr_msgs/tcu2rcu_Controller" 
;;                          ;; sane values enforce by server
;;                          :mode 4)) ; JOINT-IMPEDANCE-MODE
;;         (interpolator-msg (roslisp:make-msg 
;;                            "dlr_msgs/tcu2rcu_Interpolator"
;;                            ;; sane values enforced by server
;;                            :mode 5 ; JOINT-SCALING-INTERPOLATION
;;                            :o_t_f (to-vector (cl-transforms:make-identity-transform))
;;                            :o_t_via (to-vector (cl-transforms:make-identity-transform)))))
;;     (values controller-msg interpolator-msg)))

;; (defmethod to-msg ((params safety-settings) &key &allow-other-keys)
;;   "Creates a 'dlr_msgs/tcu2rcu_Safety' message using the data stored in `params' of type
;;  'safety-settings'."
;;   (unless (safety-settings-valid-p params)
;;     (error 'beasty-conversion-error :text "Provided safety settings were not valid."))
;;   (let ((msg
;;           (roslisp:make-msg 
;;            "dlr_msgs/tcu2rcu_Safety"
;;            :contact (roslisp:make-msg "dlr_msgs/tcu2rcu_Contact"
;;                                       :strategy (convert-strategy-vector (reflexes params))
;;                                       :threshold *default-thresholds*)
;;            :virt_env (roslisp:make-msg "dlr_msgs/tcu2rcu_VirtEnv"
;;                                        :human_ff_elements (to-msg (human params))))))
;;     (roslisp:with-fields (switches) msg
;;       (setf (elt switches 5) 128)) ; activate human-avoidance
;;     msg))

;; (defmethod to-msg ((params cl-human-shapes:human-body) &key &allow-other-keys)
;;   (let ((result (make-array 100 :initial-element 0))
;;         (loop-counter 0))
;;     (loop for body in (cl-human-shapes:body-parts params) do
;;       (setf (subseq result (* 6 loop-counter) (* 6 (incf loop-counter))) (to-msg body)))
;;     result))
    
;; (defmethod to-msg ((params cl-human-shapes:human-body-part) &key &allow-other-keys)
;;   (vector (cl-transforms:x (cl-3d-shapes:centroid (cl-human-shapes:shape params)))
;;           (cl-transforms:y (cl-3d-shapes:centroid (cl-human-shapes:shape params)))
;;           (cl-transforms:z (cl-3d-shapes:centroid (cl-human-shapes:shape params)))
;;           (cl-3d-shapes:radius (cl-human-shapes:shape params))
;;           200)) ; TODO(Georg): stiffness
   
;; (defun convert-strategy-vector (strategies)
;;   "Iterates over all strategy entries in hash-table `strategies' and transforms them into a
;;  vector of numbers. The format corresponds to the expectations of beasty in 'tcu2rcu...'."
;;   (declare (type hash-table strategies))
;;   (let ((result (make-array 8 :initial-element 0)))
;;     (loop for collision being the hash-key in strategies using (hash-value reflex) do
;;       (setf (elt result (gethash collision *collision-index-map*)) 
;;             (gethash (reaction-type reflex) *reaction-code-map*)))
;;     result))

;; (defmethod from-msg ((msg dlr_msgs-msg:rcu2tcu))
;;   "Creates and returns an instance of cram-beasty:beasty-state filled with
;;    relevant content from `msg'."
;;   (with-fields (robot safety) msg
;;     (with-fields (q power emergency o_t_x) robot
;;       (with-fields (contact_joint) safety
;;         (make-instance 'beasty-state
;;                        :motor-power-on (motor-power-flags-on-p power)
;;                        :emergency-released (emergency-flags-released-p emergency)
;;                        :joint-values q
;;                        :joint-collisions (calculate-collision-joints contact_joint)
;;                        :tcp-pose (to-transform o_t_x))))))

;; ;; TODO(Georg): move this into corresponding from-msg
;; (defun motor-power-flags-on-p (motors)
;;   "Checks whether all flags in vector `motors' indicate power-on."
;;   (declare (type vector motors))
;;   (every (lambda (motor) (> motor 0.0)) motors))

;; TODO(Georg): move this into corresponding from-msg
;; (defun emergency-flags-released-p (emergency-flags)
;;   "Checks whether all flags in vector `emergency-flags' indicate released emergency
;;  buttons."
;;   (declare (type vector emergency-flags))
;;   (every (lambda (flag) (> flag 0.0)) emergency-flags))

;; (defun calculate-collision-joints (joints &optional (prefix "/left"))
;;   "Returns vector of joint-collisions which have been reported as in collision. `joints' is
;;  feedback vector provided by Beasty as 'contact-joint' in rcu2tcu_Safety, while string
;;  `joint-prefix' is used to reconstruct the correct joint- and link-names."
;;   (declare (type vector joints)
;;            (type string prefix))
;;   (flet ((collision-p (joint)
;;            (declare (type number joint))
;;            (/= joint 0))
;;          (get-joint-name (prefix index)
;;            (declare (type string prefix)
;;                     (type number index))
;;            (concatenate 'string prefix "_arm_" (write-to-string index) "_joint"))
;;          (get-link-name (prefix index)
;;            (declare (type string prefix)
;;                     (type number index))
;;            (concatenate 'string prefix "_arm_" (write-to-string (incf index)) "_link"))
;;          (get-collision-type (collision)
;;            (multiple-value-bind (collision-type bound-p)
;;                (gethash collision *collision-symbol-map*)
;;              (if bound-p
;;                  collision-type
;;                  (error 'beasty-conversion-error 
;;                         :text "Beasty reported unknown collision type.")))))
;;     (map 'vector #'identity
;;          (loop for i from 0 to (- (length joints) 1)
;;                when (collision-p (elt joints i))
;;                  collecting (make-instance 
;;                              'collision
;;                              :joint-name (get-joint-name prefix i)
;;                              :link-name (get-link-name prefix i)
;;                              :collision-type (get-collision-type (elt joints i)))))))

;; (defmethod to-vector ((pose cl-transforms:pose))
;;   (to-vector (cl-transforms:make-transform
;;               (cl-transforms:origin pose)
;;               (cl-transforms:orientation pose))))

;; (defmethod to-vector ((transform cl-transforms:transform))
;;   "Turns 'cl-transforms:transform' `transform' into row-ordered 16x1 double-array."
;;   (let* ((array4x4 (cl-transforms:transform->matrix transform))
;;          (array4x4t (make-array 
;;                      '(4 4) 
;;                      :initial-contents
;;                      (loop for x from 0 below 4 collecting
;;                                                 (loop for y from 0 below 4 collecting
;;                                                                            (aref array4x4 y x))))))
;;     (make-array (array-total-size array4x4t)
;;                 :element-type (array-element-type array4x4t)
;;                 :displaced-to array4x4t)))

;; (defmethod to-vector ((translation cl-transforms:3d-vector))
;;   "Turns 'cl-transforms:3d-vector' `translation' into 3x1 array."
;;   (vector (cl-transforms:x translation)
;;           (cl-transforms:y translation)
;;           (cl-transforms:z translation)))

;; (defmethod to-vector ((point cl-transforms:3d-vector))
;;   "Turns 'cl-transforms:3d-vector' `point' into a regular array of size 3."
;;   (vector (cl-transforms:x point) (cl-transforms:y point) (cl-transforms:z point)))

;; (defun to-transform (input-vector)
;;   "Returns an instance of type 'cl-transforms:transforms' corresponding to the content
;;  of `input-vector'. Where `input-vector' is a vector of length 12 expected to have the
;;  following layout: #(rotation-matrix-in-column-major translation-vector)."
;;            (declare (type (vector number 12) input-vector))
;;            (let* ((rotation-vector (subseq input-vector 0 9))
;;                   (translation-vector (subseq input-vector 9 12))
;;                   (rotation-matrix 
;;                     (make-array 
;;                      '(3 3) 
;;                      :initial-contents
;;                      (loop for x from 0 below 3
;;                            collecting (loop for y from 0 below 3
;;                                             collecting (elt rotation-vector
;;                                                             (+ x (* 3 y)))))))
;;                   (quaternion (cl-transforms:matrix->quaternion rotation-matrix))
;;                   (translation (to-point translation-vector)))
;;              (cl-transforms:make-transform translation quaternion)))

;; (defun to-point (input-vector)
;;   "Returns an instance of type 'cl-transforms:3d-vector' corresponding to the content of
;; `input-vector' which is expected to be a vector with 3 numbers."
;;   (declare (type (vector number 3) input-vector))
;;   (cl-transforms:make-3d-vector (elt input-vector 0) 
;;                                 (elt input-vector 1) 
;;                                 (elt input-vector 2)))