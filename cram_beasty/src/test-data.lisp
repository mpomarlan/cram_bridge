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
     :tool-com ,(cl-transforms:make-identity-vector)
     :session-id 123
     :cmd-id 456))

(defparameter *sample-handle*
  (make-instance 
   'beasty-handle :session-id 123 :cmd-id 456))