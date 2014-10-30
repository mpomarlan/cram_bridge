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

(defparameter *joint-symbols* 
  (list :joint0 :joint1 :joint2 :joint3 :joint4 :joint5 :joint6))

(defparameter *joint-indices*
  (list 0 1 2 3 4 5 6))

(defparameter *joint-index-map*
  (pairlis *joint-symbols* *joint-indices*))

(defparameter *cartesian-symbols*
  (list :trans-x :trans-y :trans-z :rot-x :rot-y :rot-z))

(defparameter *cartesian-indices*
  (list 0 1 2 3 4 5))

(defparameter *cartesian-index-map*
  (pairlis *cartesian-symbols* *cartesian-indices*))

(defparameter *joint-goal-attribute-symbols*
  '(:goal-pos :max-vel :max-acc :stiffness :damping))

(defparameter *cartesian-goal-attribute-symbols*
  '(:max-vel :max-acc :stiffness :damping))

(defparameter *cartesian-vector-attribute-symbols*
  (mapcar (alexandria:curry #'prefix-keyword "cartesian-") *cartesian-goal-attribute-symbols*))

(defparameter *joint-vector-attribute-symbols*
  (mapcar (alexandria:curry #'prefix-keyword "joint-") *joint-goal-attribute-symbols*))