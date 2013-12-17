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

(defclass reset-safety-parameters () ()
  (:documentation "Class signalling to reset the safety brakes of a Beasty controller."))