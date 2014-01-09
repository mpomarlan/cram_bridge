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

;;; BEASTY INTERFACE

(defclass beasty-interface ()
  ((action-client :initarg :action-client :reader action-client 
                  :type actionlib::action-client
                  :documentation "For internal use. ROS action client to communicate with controller.")
   (session-id :initarg :session-id :accessor session-id :type number
               :documentation "For internal use. ID of current communication session.")
   (cmd-id :initarg :cmd-id :accessor cmd-id :type number
           :documentation "For internal use. cmd-id to be used in the next goal.")
   (state-sub :initform nil :accessor state-sub
              :documentation "For internal use. Subscriber listening to state-topic of server.")
   (visualization-pub :initarg :visualization-pub :accessor visualization-pub
                      :type publication 
                      :documentation "For internal use. Publisher for visualization.")
   (state :initform (cram-language:make-fluent :value (make-instance 'beasty-state))
          :accessor state :type cram-language:value-fluent
          :documentation "Fluent with last state reported from beasty controller.")
   (robot :initform (make-instance 'beasty-robot) :accessor robot :type beasty-robot
          :documentation "Robot representation of LWR controlled by this interface."))
  (:documentation "Action-client interface with book-keeping for LWR controller Beasty."))

(defclass beasty-state ()
  ((motor-power-on :initarg :motor-power-on :reader motor-power-on :type boolean
                   :documentation "Indicates whether Beasty has all motors powered on.")
   (safety-released :initarg :safety-released :reader safety-released :type boolean
                    :documentation "Indicates whether safety buttons are released.")
   (joint-values :initarg :joint-values :reader joint-values
                 :type vector :documentation "Current joint values of LWR arm.")
   (tcp-pose :initarg :tcp-pose :reader tcp-pose :type cl-transforms:transform
              :documentation "Pose of tcp frame w.r.t. to arm base frame.")
   (joint-contacts :initarg :joint-contacts :reader joint-contacts
                   :type vector :documentation "Detected contacts per joint of LWR arm.")
   (joint-collisions :initarg :joint-collisions :reader joint-collisions
                     :type vector :documentation "Detected collisions per joint of LWR."))
  (:documentation "Representation of state reported from Beasty LWR controller."))

;;; CONTROLLER PARAMETERS

(defclass gravity-control-parameters ()
  ((max-joint-vel :initform (make-array 7 :initial-element 0.35) :accessor max-joint-vel
                  :type vector :documentation "Maximum joint velocities in rad/s.")
   (max-joint-acc :initform (make-array 7 :initial-element 0.7) :accessor max-joint-acc
                  :type vector :documentation "Maximum joint accelerations in rad/s^2."))
  (:documentation "Class holding all parameters necessary to configure gravity compensation
    control mode of the Beasty controller."))

(defclass joint-impedance-control-parameters ()
  ((joint-goal :initform (make-array 7 :initial-element 0.0) :accessor joint-goal
               :type vector :documentation "Joint-space goal vector in radians.")
   (joint-stiffness :initform (make-array 7 :initial-element 20.0) 
                    :accessor joint-stiffness :type vector
                    :documentation "Joint stiffness vector. Range: 0...2000.")
   (joint-damping :initform (make-array 7 :initial-element 0.7) :accessor joint-damping
                  :type vector :documentation "Joint damping vector. Range 0...1.0. User
                  may select critically-damped behavior with a value of 0.7.")
   (max-joint-vel :initform (make-array 7 :initial-element 0.35) :accessor max-joint-vel
                  :type vector :documentation "Maximum joint velocities in rad/s.")
   (max-joint-acc :initform (make-array 7 :initial-element 0.7) :accessor max-joint-acc
                  :type vector :documentation "Maximum joint accelerations in rad/s^2."))
  (:documentation "Class holding all parameters necessary to configure joint impedance
   control mode of the Beasty controller."))

(defclass cartesian-impedance-control-parameters ()
  ((cart-stiffness :initform (vector 1500 1500 1500 200 200 200) :accessor
                   cart-stiffness :type vector :documentation "Cartesian stiffness of a
 Cartesian impedance motion, defined w.r.t. EE frame. Order: t_x, t_y, t_z, r_x, r_y,
 r_z. Range for position: 0..4000. Range for orientations: 0..400.")
   (cart-damping :initform (make-array 6 :initial-element 0.7) :accessor cart-damping
                 :type vector :documentation "Cartesian damping of a Cartesian impedance
 motion. Order t_x, t_y, t_z, r_x, r_y, r_z. Range: 0..1; 0.7 means critically-damped.")
   (nullspace-stiffness :initform 20.0 :accessor nullspace-stiffness :type number
                        :documentation "Stiffness of nullspace (elbow) controller of
 Cartesian impedance motion for Beasty, defined w.r.t. ???. Range: 0..100.")
   (nullspace-damping :initform 0.7 :accessor nullspace-damping :type number
                      :documentation "Damping of nullspace (elbow) controller of Cartesian
 impedance motion for Beasty. Range: 0..1, 0.7 means critically-damped.")
   (nullspace-dir :initform (cl-transforms:make-identity-vector) :accessor nullspace-dir
                  :type cl-transforms:3d-vector :documentation "Nullspace (elbow) controller
 direction of Cartesian impedance motion for Beasty, defined w.r.t. base-frame.")
   (filter-gains :initform (make-array 6 :initial-element 0.0) :accessor filter-gains
                 :type vector :documentation "Gains for filtering of cartesian-stiffnesses.
Range for positions: 0..400000. Range for orientations: 0..40000.")
   (goal-pose :initform (cl-transforms:make-identity-transform) :accessor goal-pose
              :type cl-transforms:transform :documentation "Goal pose for EE frame of
 Cartesian impedance motion of Beasty, defined w.r.t. base frame of LWR.")
   (max-cart-vel :initform (vector 0.1 0.1 0.1 0.3 0.3 0.3) :accessor max-cart-vel
                 :type vector :documentation "Maximum Cartesian velocities of Cartesian
 impedance motion of Beasty, defined w.r.t. arm base. Order: t_x, t_y, t_z, r_x, r_y, r_z")
   (max-cart-acc :initform (vector 0.2 0.2 0.2 0.6 0.6 0.6) :accessor max-cart-acc
                 :type vector :documentation "Maximum Cartesian accelerations of Cartesian
impedance motion of Beasty, defined w.r.t. arm base. Order: t_x, t_y, t_z, r_x, r_y, r_z."))
  (:documentation "Class holding all parameters necessary to configure Cartesian impedance 
   control mode of the Beasty controller."))

(defclass reset-safety-parameters () ()
  (:documentation "Class to release the software safety flags of a Beasty controller."))

;;; ROBOT MODELLING

(defclass beasty-robot ()
  ((simulation-flag :initform t :accessor simulation-flag :type boolean
                    :documentation "Indicates simulated robot. 'nil' for real robot.")
   (motor-power :initform nil :accessor motor-power :type boolean
                :documentation "Flag for power of motors. nil=power-off, t:power-on.")
   (tool-configuration :initform (make-instance 'beasty-tool) 
                       :accessor tool-configuration :type beasty-tool
                       :documentation "Description of the EE mounted on the arm.")
   (base-configuration :initform (make-instance 'beasty-base) 
                       :accessor base-configuration :type beasty-base
                       :documentation "Description of the mounting of the arm's base."))
  (:documentation "Representation of LWR robot for Beasty controller."))

(defclass beasty-base ()
  ((base-transform :initform (cl-transforms:make-identity-transform)
                   :accessor base-transform :type cl-transforms:transform
                   :documentation "Transform from World to Base. Note: Base is located in the base of the LWR with z-axis pointing to 1st joint and the x-axis pointing to the cable connection. World may be chosen arbitrarily.")
   (base-acceleration :initform (make-array 6 :initial-element 0)
                      :accessor base-acceleration :type vector
                      :documentation "6-dimensional Cart. acceleration acting on the base of the robot."))
  (:documentation "Representation of base configuration of LWR controlled by Beasty."))

(defclass beasty-tool ()
  ((ee-transform :initform (cl-transforms:make-identity-transform)
                 :accessor ee-transform :type cl-transforms:transform
                 :documentation "Transform from TCP to EE. Note: TCP is located at the center of the last link (sphere) of the LWR III with the z-axis pointing in the direction to the flange. For q7 = 0 the y-axis points to the 6th joint.")
   (mass :initform 0.0 :accessor mass :type number
         :documentation "Mass in kg of the EE (incl. load).")
   (com :initform (cl-transforms:make-identity-vector) :accessor com 
        :type cl-transforms:3d-vector :documentation "Center of mass of EE w.r.t. to TCP."))
  (:documentation "Representation of tool mounted on LWR controlled by Beasty."))