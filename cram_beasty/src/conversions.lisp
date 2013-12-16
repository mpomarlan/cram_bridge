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
  (let ((robot-msg
          (roslisp:make-msg "dlr_msgs/tcu2rcu_Robot"
                            :mode (if (simulation-flag robot) 1 0)
                            :power (if (motor-power robot)
                                       (make-array 7 :initial-element 1)
                                       (make-array 6 :initial-element 0))))
        (settings-msg
          (roslisp:make-msg "dlr_msgs/tcu2rcu_Settings"
                            :tcp_t_ee (to-vector (ee-transform (tool-configuration robot)))
                            :w_t_o (to-vector (base-transform (base-configuration robot)))
                            :w_t_op (to-vector (cl-transforms:make-identity-transform))
                            :ee_t_k (to-vector (cl-transforms:make-identity-transform))
                            :ref_t_k (to-vector (cl-transforms:make-identity-transform))
                            :ml (mass (tool-configuration robot))
                            :ml_com (to-vector (com (tool-configuration robot)))
                            :ddx_o (base-acceleration (base-configuration robot)))))
    (values robot-msg settings-msg)))

(defmethod to-msg ((params gravity-control-parameters))
  (let ((controller-msg (roslisp:make-msg "dlr_msgs/tcu2rcu_Controller" :mode 3))
        (interpolator-msg (roslisp:make-msg 
                           "dlr_msgs/tcu2rcu_Interpolator"
                           :mode 5 ; JOINT-SCALING-INTERPOLATION
                           :dq_max (max-joint-vel params)
                           :ddq_max (max-joint-acc params)
                           :o_t_f (to-vector (cl-transforms:make-identity-transform))
                           :o_t_via (to-vector (cl-transforms:make-identity-transform)))))
    (values controller-msg interpolator-msg)))

(defmethod to-vector ((transform cl-transforms:transform))
  (let ((array4x4 (cl-transforms:transform->matrix transform)))
    (make-array (array-total-size array4x4)
                :element-type (array-element-type array4x4)
                :displaced-to array4x4)))

(defmethod to-vector ((point cl-transforms:3d-vector))
  (vector (cl-transforms:x point) (cl-transforms:y point) (cl-transforms:z point)))