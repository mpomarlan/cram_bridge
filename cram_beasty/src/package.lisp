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

(in-package :cl-user)

(defpackage :cram-beasty
  (:use #:roslisp
        #:common-lisp
        #:sb-thread
        #:actionlib-lisp
        )
  ;; (:export 
  ;;  ;; top-level interaction with beasty interface
  ;;  :make-beasty-interface :cleanup-beasty-interface :beasty-interface :command-beasty
  ;;  :cancel-command :robot :state :stop-beasty
  ;;  ;; modelling of LWR robot for beasty
  ;;  :beasty-robot :simulation-flag :tool-configuration :base-configuration
  ;;  :beasty-base :base-transform :base-acceleration :base-frame-id :beasty-tool
  ;;  :ee-transform :mass :com :collision :joint-name :collision-type :link-name
  ;;  ;; commanding gravity compensation parameters
  ;;  :gravity-control-parameters :max-joint-vel :max-joint-acc
  ;;  ;; commanding joint impedance parameters
  ;;  :make-joint-impedance-goal :joint-impedance-control-parameters :joint-goal
  ;;  :joint-stiffness :joint-damping
  ;;  ;; commanding cartesian impedance parameters
  ;;  :cartesian-impedance-control-parameters :cart-stiffness :cart-damping
  ;;  :nullspace-stiffness :nullspace-damping :nullspace-dir :filter-gains
  ;;  :goal-pose :max-cart-vel :max-cart-acc
  ;;  ;; commanding a complete stop with brakes kicking in
  ;;  :hard-stop-parameters
  ;;  ;; state feedback coming from controller
  ;;  :beasty-state :motor-power-on :emergency-released :joint-values :tcp-pose
  ;;  :get-strongest-collision
  ;;  ;; safety settings
  ;;  :make-safety-settings :store-reflex :remove-reflex :safety-settings-valid-p
  ;;  :make-beasty-reflex :human
  ;;  ;; collision types
  ;;  :NO-COLLISION :CONTACT :LIGHT-COLLISION :STRONG-COLLISION :SEVERE-COLLISION
  ;;  ;; reaction types
  ;;  :IGNORE :ZERO-G :JOINT-IMP :SOFT-STOP :HARD-STOP)
  )
