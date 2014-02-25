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

(in-package :cram-fccl)

;; (defclass geometric-feature ()
;;   ((name :initarg :name :reader name :type string)
;;    (reference-id :initarg :reference-id :reader reference-id :type string)
;;    (feature-type :initarg :feature-type :reader feature-type)
;;    (feature-position :initarg :feature-position :reader feature-position)
;;    (feature-direction :initarg :feature-direction :reader feature-direction)))

;; (defclass geometric-constraint ()
;;   ((name :initarg :name :reader name :type string)
;;    (reference-id :initarg :reference-id :reader reference-id :type string)
;;    (constraint-function :initarg :constraint-function 
;;                         :reader constraint-function :type string)
;;    (tool-feature :initarg :tool-feature :reader tool-feature)
;;    (object-feature :initarg :object-feature :reader object-feature)
;;    (lower-boundary :initarg :lower-boundary :reader lower-boundary)
;;    (upper-boundary :initarg :upper-boundary :reader upper-boundary)))

;; (defclass geometric-constraint-state ()
;;   ((output :initarg :output :reader output)
;;    (desired-output :initarg :desired-output :reader desired-output)
;;    (weight :initarg :weight :reader weight)))

;; (defclass geometric-constraint-feedback ()
;;   ((command :initarg :command :reader command :type geometric-constraint)
;;    (output :initarg :output :reader output :type geometric-constraint-state)))

(defclass kinematic-chain ()
  ((base-frame-id :initarg :base-frame-id :reader base-frame-id :type string)
   (tip-frame-id :initarg :tip-frame-id :reader tip-frame-id :type string)))

;; (defun make-geometric-feature (name reference-id type position 
;;                                &optional (direction (cl-transforms:make-identity-vector)))
;;   (make-instance 'geometric-feature
;;                  :name name
;;                  :reference-id reference-id
;;                  :feature-type type
;;                  :feature-position position
;;                  :feature-direction direction))

;; (defun make-point-feature (name reference-id position)
;;   (make-geometric-feature name reference-id 'point position))

;; (defun make-line-feature (name reference-id position direction)
;;   (make-geometric-feature name reference-id 'line position direction))

;; (defun make-plane-feature (name reference-id position direction)
;;   (make-geometric-feature name reference-id 'plane position direction))

;; (defun make-geometric-constraint (name reference-id function 
;;                                   tool-feature object-feature
;;                                   lower-boundary upper-boundary)
;;   (declare (type string name reference-id function)
;;            (type geometric-feature tool-feature object-feature)
;;            (type number lower-boundary upper-boundary))
;;   (make-instance 'geometric-constraint
;;                  :name name
;;                  :reference-id reference-id
;;                  :constraint-function function
;;                  :tool-feature tool-feature
;;                  :object-feature object-feature
;;                  :lower-boundary lower-boundary
;;                  :upper-boundary upper-boundary))

;; (defun make-geometric-constraint-state (output desired-output weight)
;;   (make-instance 'geometric-constraint-state
;;                  :output output :desired-output desired-output :weight weight))

;; (defun make-geometric-constraint-feedback (command output)
;;   (make-instance 'geometric-constraint-feedback :command command :output output))

(defun make-kinematic-chain (base-frame-id tip-frame-id)
  (declare (type string base-frame-id tip-frame-id))
  (make-instance 'kinematic-chain
                 :base-frame-id base-frame-id
                 :tip-frame-id tip-frame-id))