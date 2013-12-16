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
                       :documentation "Description of the mounting of the arm's base.")))

(defclass beasty-base ()
  ((base-transform :initform (cl-transforms:make-identity-transform)
                   :accessor base-transform :type cl-transforms:transform
                   :documentation "Transform from World to Base. Note: Base is located in the base of the LWR with z-axis pointing to 1st joint and the x-axis pointing to the cable connection. World may be chosen arbitrarily.")
   (base-acceleration :initform (make-array 6 :initial-element 0)
                      :accessor base-acceleration :type vector
                      :documentation "6-dimensional Cart. acceleration acting on the base of the robot.")))
   

(defclass beasty-tool ()
  ((ee-transform :initform (cl-transforms:make-identity-transform)
                 :accessor ee-transform :type cl-transforms:transform
                 :documentation "Transform from TCP to EE. Note: TCP is located at the center of the last link (sphere) of the LWR III with the z-axis pointing in the direction to the flange. For q7 = 0 the y-axis points to the 6th joint.")
   (mass :initform 0.0 :accessor mass :type number
         :documentation "Mass in kg of the EE (incl. load).")
   (com :initform (cl-transforms:make-identity-vector) :accessor com 
        :type cl-transforms:3d-vector :documentation "Center of mass of EE w.r.t. to TCP.")))