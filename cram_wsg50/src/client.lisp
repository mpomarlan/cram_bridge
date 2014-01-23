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

(in-package :cram-wsg50)

(defclass wsg50-interface () ;; TODO(Georg): add type-declarations
  ((open-client :initarg :open-client :accessor open-client
                 :documentation "ROS service client to command the gripper to open its
                 fingers.")
   (close-client :initarg :close-client :accessor close-client
                 :documentation "ROS service client to command the gripper to close its
                 fingers.")
   (homing-client :initarg :homing-client :accessor homing-client
                  :documentation "ROS service client to home the gripper.")
   (status-subscriber :accessor status-subscriber
                      :documentation "ROS topic subscriber to status topic of gripper.")
   (status :accessor status :type wsg50-status
           :documentation "For internal use. Last reported status of gripper."))
  (:documentation "ROS Interface talking to Schunk WSG50 gripper controller."))

(defclass wsg50-status ()
  ((width :initarg :width :reader width :type number
          :documentation "Width opening of Schunk WSG50 gripper.")
   (max-acc :initarg :max-acc :reader max-acc :type number
            :documentation "Maximum acceleration setting of Schunk WSG50 gripper.")
   (max-force :initarg :max-force :reader max-force :type number
              :documentation "Maximum force setting of Schunk WSG50 gripper."))
  (:documentation "For internal use. Gripper status of Schunk WSG50."))