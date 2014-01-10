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

(defgeneric to-msg (data)
  (:documentation "Creates the ROS message corresponding to lisp-data `data'."))

(defgeneric from-msg (msg)
  (:documentation "Creates the lisp data structure corresponding to ROS message `msg'."))

(defgeneric to-vector (data)
  (:documentation "Transforms `data' into a corresponding vector representation."))

(defmethod to-msg ((robot beasty-robot))
  "Creates 'dlr_msgs/tcu2rcu_Robot' and 'dlr_msgs/tcu2rcu_Settings' message using the data
   stored in `robot'."
  (let ((robot-msg
          (roslisp:make-msg "dlr_msgs/tcu2rcu_Robot"
                            :mode (if (simulation-flag robot) 1 0)
                            :power (if (emergency-released-flag robot)
                                        (make-array 7 :initial-element 1)
                                        (make-array 7 :initial-element 0))))
        (settings-msg
          (roslisp:make-msg "dlr_msgs/tcu2rcu_Settings"
                            :tcp_t_ee (to-vector (ee-transform (tool-configuration robot)))
                            :w_t_o (to-vector (base-transform (base-configuration robot)))
                            ;; Cart. velocities for Cartesian velocity control are
                            ;; interpreted w.r.t world-frame
                            :w_t_op (to-vector (cl-transforms:make-identity-transform))
                            ;; Cartesian Impedances are interpreted w.r.t to EE frame
                            :ee_t_k (to-vector (cl-transforms:make-identity-transform))
                            ;; sane value enforced by beasty
                            :ref_t_k (to-vector (cl-transforms:make-identity-transform))
                            :ml (mass (tool-configuration robot))
                            :ml_com (to-vector (com (tool-configuration robot)))
                            :ddx_o (base-acceleration (base-configuration robot)))))
    (values robot-msg settings-msg)))

(defmethod to-msg ((params gravity-control-parameters))
  "Creates a 'dlr_msgs/tcu2rcu_Controller' and a 'dlr_msgs/tcu2rcu_Interpolator' message
   using the data stored in `params' of type 'gravity-control-parameters'."
  (let ((controller-msg (roslisp:make-msg "dlr_msgs/tcu2rcu_Controller" :mode 3))
        (interpolator-msg (roslisp:make-msg 
                           "dlr_msgs/tcu2rcu_Interpolator"
                           :mode 5 ; JOINT-SCALING-INTERPOLATION
                           :dq_max (max-joint-vel params)
                           :ddq_max (max-joint-acc params)
                           ;; sane values enforced by server
                           :o_t_f (to-vector (cl-transforms:make-identity-transform))
                           :o_t_via (to-vector (cl-transforms:make-identity-transform)))))
    (values controller-msg interpolator-msg)))

(defmethod to-msg ((params joint-impedance-control-parameters))
  "Creates a 'dlr_msgs/tcu2rcu_Controller' and a 'dlr_msgs/tcu2rcu_Interpolator' message
   using the data stored in `params' of type 'joint-impedance-control-parameters'."
  (let ((controller-msg (roslisp:make-msg 
                         "dlr_msgs/tcu2rcu_Controller" 
                         :mode 4 ; JOINT-IMPEDANCE-MODE
                         :k_theta (joint-stiffness params)
                         :d_theta (joint-damping params)))
        (interpolator-msg (roslisp:make-msg 
                           "dlr_msgs/tcu2rcu_Interpolator"
                           :mode 5 ; JOINT-SCALING-INTERPOLATION
                           :dq_max (max-joint-vel params)
                           :ddq_max (max-joint-acc params)
                           :q_f (joint-goal params)
                           ;; sane values enforced by server
                           :o_t_f (to-vector (cl-transforms:make-identity-transform))
                           :o_t_via (to-vector (cl-transforms:make-identity-transform)))))
    (values controller-msg interpolator-msg)))

(defmethod to-msg ((params cartesian-impedance-control-parameters))
  "Creates a 'dlr_msgs/tcu2rcu_Controller' and a 'dlr_msgs/tcu2rcu_Interpolator' message
   using the data stored in `params' of type 'cartesian-impedance-control-parameters'."
  (let ((controller-msg (roslisp:make-msg 
                         "dlr_msgs/tcu2rcu_Controller" 
                         :mode 2 ; CARTESIAN-IMPEDANCE-MODE
                         :k_x (cart-stiffness params)
                         :d_x (cart-damping params)
                         :k_ns (nullspace-stiffness params)
                         :xi_ns (nullspace-damping params)
                         :w_ns_dir (to-vector (nullspace-dir params))
                         :enable_ffwd t
                         :dk_x (filter-gains params)))
        (interpolator-msg (roslisp:make-msg 
                           "dlr_msgs/tcu2rcu_Interpolator"
                           :mode 4 ; CARTESIAN-RAMP
                           :o_t_f (to-vector (goal-pose params))
                           :dx_max (max-cart-vel params)
                           :ddx_max (max-cart-acc params)
                           ; sane values enforced by Beasty
                           :o_t_via (to-vector (cl-transforms:make-identity-transform)))))
    (values controller-msg interpolator-msg)))

;; TODO(Georg): the next three are identical; find a way to re-use the methods.
(defmethod to-msg ((params reset-emergency-parameters))
  "Creates a 'dlr_msgs/tcu2rcu_Controller' and a 'dlr_msgs/tcu2rcu_Interpolator' message
   using the data stored in `params' of type 'reset-emergency-parameters'."
  (let ((controller-msg (roslisp:make-msg 
                         "dlr_msgs/tcu2rcu_Controller" 
                         ;; sane values enforce by server
                         :mode 4)) ; JOINT-IMPEDANCE-MODE
        (interpolator-msg (roslisp:make-msg 
                           "dlr_msgs/tcu2rcu_Interpolator"
                           ;; sane values enforced by server
                           :mode 5 ; JOINT-SCALING-INTERPOLATION
                           :o_t_f (to-vector (cl-transforms:make-identity-transform))
                           :o_t_via (to-vector (cl-transforms:make-identity-transform)))))
    (values controller-msg interpolator-msg)))

(defmethod to-msg ((params hard-stop-parameters))
  "Creates a 'dlr_msgs/tcu2rcu_Controller' and a 'dlr_msgs/tcu2rcu_Interpolator' message
   using the data stored in `params' of type 'complete-stop-parameters'."
  (let ((controller-msg (roslisp:make-msg 
                         "dlr_msgs/tcu2rcu_Controller" 
                         ;; sane values enforce by server
                         :mode 4)) ; JOINT-IMPEDANCE-MODE
        (interpolator-msg (roslisp:make-msg 
                           "dlr_msgs/tcu2rcu_Interpolator"
                           ;; sane values enforced by server
                           :mode 5 ; JOINT-SCALING-INTERPOLATION
                           :o_t_f (to-vector (cl-transforms:make-identity-transform))
                           :o_t_via (to-vector (cl-transforms:make-identity-transform)))))
    (values controller-msg interpolator-msg)))

(defmethod to-msg ((params safety-reset))
  "Creates a 'dlr_msgs/tcu2rcu_Controller' and a 'dlr_msgs/tcu2rcu_Interpolator' message
   using the data stored in `params' of type 'safety-reset'."
  (let ((controller-msg (roslisp:make-msg 
                         "dlr_msgs/tcu2rcu_Controller" 
                         ;; sane values enforce by server
                         :mode 4)) ; JOINT-IMPEDANCE-MODE
        (interpolator-msg (roslisp:make-msg 
                           "dlr_msgs/tcu2rcu_Interpolator"
                           ;; sane values enforced by server
                           :mode 5 ; JOINT-SCALING-INTERPOLATION
                           :o_t_f (to-vector (cl-transforms:make-identity-transform))
                           :o_t_via (to-vector (cl-transforms:make-identity-transform)))))
    (values controller-msg interpolator-msg)))

(defmethod to-msg ((params safety-settings))
  "Creates a 'dlr_msgs/tcu2rcu_Safety' message using the data stored in `params' of type
 'safety-settings'."
  (declare (ignore params))
  (roslisp:make-msg 
   "dlr_msgs/tcu2rcu_Safety"
   :contact (roslisp:make-msg "dlr_msgs/tcu2rcu_Contact"
                              ;; TODO(Georg): make this a global parameter
                              ;; TODO(Georg): add strategy vector
                              :threshold #(0.08 0.15 0.25 0.35 0.35 0.35 0.35 0.35))))

(defmethod from-msg ((msg dlr_msgs-msg:rcu2tcu))
  "Creates and returns an instance of cram-beasty:beasty-state filled with
   relevant content from `msg'."
  (with-fields (robot safety) msg
    (with-fields (q power emergency o_t_x) robot
      (with-fields (contact_joint) safety
        (make-instance 'beasty-state
                       :motor-power-on (motor-power-flags-on-p power)
                       :emergency-released (emergency-flags-released-p emergency)
                       :joint-values q
                       :joint-collisions (calculate-collision-joints contact_joint)
                       :tcp-pose (to-transform o_t_x))))))

(defun motor-power-flags-on-p (motors)
  "Checks whether all flags in vector `motors' indicate power-on."
  (declare (type vector motors))
  (every (lambda (motor) (> motor 0.0)) motors))

(defun emergency-flags-released-p (emergency-flags)
  "Checks whether all flags in vector `emergency-flags' indicate released emergency
 buttons."
  (declare (type vector emergency-flags))
  (every (lambda (flag) (> flag 0.0)) emergency-flags))

(defun calculate-collision-joints (joints &optional (joint-prefix "/left"))
  "Returns vector of joint-names which have been reported as in collision. `joint' is
 feedback vector provided by Beasty as 'collision-joint' in rcu2tcu_Safety, while string
 `joint-prefix' is used to reconstruct the correct joint-names."
  (declare (type vector joints)
           (type string joint-prefix))
  (coerce
   (loop for i from 0 to (- (length joints) 1)
         when (joint-collision-p (elt joints i))
           collecting (construct-joint-name joint-prefix i))
   'vector))

(defun calculate-collision-joints2 (joints &optional (prefix "/left"))
  "Returns vector of joint-collisions which have been reported as in collision. `joints' is
 feedback vector provided by Beasty as 'contact-joint' in rcu2tcu_Safety, while string
 `joint-prefix' is used to reconstruct the correct joint- and link-names."
  (declare (type vector joints)
           (type string prefix))
  (flet ((collision-p (joint)
           (declare (type number joint))
           (/= joint 0))
         (get-joint-name (prefix index)
           (declare (type string prefix)
                    (type number index))
           (concatenate 'string prefix "_arm_" (write-to-string index) "_joint"))
         (get-link-name (prefix index)
           (declare (type string prefix)
                    (type number index))
           (concatenate 'string prefix "_arm_" (write-to-string (incf index)) "_link"))
         (get-collision-type (collision)
           (ecase collision
             (0 :NONE)
             (1 :CONTACT)
             (2 :LIGHT-COLLISION)
             (3 :STRONG-COLLISION)
             (4 :SEVERE-COLLISION))))
    (map 'vector #'identity
         (loop for i from 0 to (- (length joints) 1)
               when (collision-p (elt joints i))
                 collecting (make-instance 
                             'collision
                             :joint-name (get-joint-name prefix i)
                             :link-name (get-link-name prefix i)
                             :collision-type (get-collision-type (elt joints i)))))))
  
(defun construct-joint-name (joint-prefix joint-index)
  "Returns the joint-name corresponding to joint with `joint-index' using `joint-prefix'.
`joint-index' is supposed to be a number, and `joint-prefix' expected to be a string."
  (declare (type string joint-prefix)
           (type number joint-index))
  (concatenate 'string joint-prefix "_arm_" (write-to-string joint-index) "_joint"))

(defun joint-collision-p (joint)
  "Checks whether `joint' reported from Beasty as 'collision_joint' indicates a collision."
  (declare (type number joint))
  (/= joint 0))
    
(defmethod to-vector ((transform cl-transforms:transform))
  "Turns 'cl-transforms:transform' `transform' into row-ordered 16x1 double-array."
  (let* ((array4x4 (cl-transforms:transform->matrix transform))
         (array4x4t (make-array 
                     '(4 4) 
                     :initial-contents
                     (loop for x from 0 below 4 collecting
                                                (loop for y from 0 below 4 collecting
                                                                           (aref array4x4 y x))))))
    (make-array (array-total-size array4x4t)
                :element-type (array-element-type array4x4t)
                :displaced-to array4x4t)))

(defmethod to-vector ((translation cl-transforms:3d-vector))
  "Turns 'cl-transforms:3d-vector' `translation' into 3x1 array."
  (vector (cl-transforms:x translation)
          (cl-transforms:y translation)
          (cl-transforms:z translation)))

(defmethod to-vector ((point cl-transforms:3d-vector))
  "Turns 'cl-transforms:3d-vector' `point' into a regular array of size 3."
  (vector (cl-transforms:x point) (cl-transforms:y point) (cl-transforms:z point)))

(defun to-transform (input-vector)
  "Returns an instance of type 'cl-transforms:transforms' corresponding to the content
 of `input-vector'. Where `input-vector' is a vector of length 12 expected to have the
 following layout: #(rotation-matrix-in-row-major translation-vector)."
           (declare (type (vector number 12) input-vector))
           (let* ((rotation-vector (subseq input-vector 0 9))
                  (translation-vector (subseq input-vector 9 12))
                  (rotation-matrix 
                    (make-array 
                     '(3 3) 
                     :initial-contents
                     (loop for y from 0 below 3
                           collecting (loop for x from 0 below 3
                                            collecting (elt rotation-vector
                                                            (+ x (* 3 y)))))))
                  (quaternion (cl-transforms:matrix->quaternion rotation-matrix))
                  (translation (to-point translation-vector)))
             (cl-transforms:make-transform translation quaternion)))

(defun to-point (input-vector)
  "Returns an instance of type 'cl-transforms:3d-vector' corresponding to the content of
`input-vector' which is expected to be a vector with 3 numbers."
  (declare (type (vector number 3) input-vector))
  (cl-transforms:make-3d-vector (elt input-vector 0) 
                                (elt input-vector 1) 
                                (elt input-vector 2)))