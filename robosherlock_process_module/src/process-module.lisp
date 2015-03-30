;;; Copyright (c) 2013, Jan Winkler <winkler@cs.uni-bremen.de>
;;; All rights reserved.
;;; 
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;; 
;;;     * Redistributions of source code must retain the above copyright
;;;       notice, this list of conditions and the following disclaimer.
;;;     * Redistributions in binary form must reproduce the above copyright
;;;       notice, this list of conditions and the following disclaimer in the
;;;       documentation and/or other materials provided with the distribution.
;;;     * Neither the name or Universitaet Bremen nor the names of its
;;;       contributors may be used to endorse or promote products derived from
;;;       this software without specific prior written permission.
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

(in-package :robosherlock-process-module)

(defparameter *object-marker-color* `(1.0 0.0 0.0 1.0))

(defgeneric call-action (action &rest params))

(defmacro def-action-handler (name args &body body)
  (alexandria:with-gensyms (action-sym params)
    `(defmethod call-action ((,action-sym (eql ',name)) &rest ,params)
       (destructuring-bind ,args ,params ,@body))))

(defun newest-non-effective (designator)
  (cond ((and designator
              (parent designator)
              (desig:effective designator))
         (newest-non-effective (parent designator)))
        (t designator)))

(def-action-handler perceive (object-designator)
  (ros-info (perception) "Perceiving object.")
  (let ((results (perceive-object-designator object-designator)))
    (unless results
      ;; NOTE(winkler): If it was already effective, rewind the
      ;; designator to the last, non-effective one when no object
      ;; could be found. This helps to avoid referencing object
      ;; identifiers that were present in the environment
      ;; representation prior to the removal of disappeared
      ;; objects. If those IDs stay in the designator, they are being
      ;; looked up in the environment representation (aka bullet
      ;; world) and cause major problems when generating costmaps `(to
      ;; see)' and `(to reach)'. IMO, this should go somewhere in the
      ;; plan library (possibly `perceive-object all') and should not
      ;; be part of the (interchangeable) process modules.
      (when (desig:effective object-designator)
        (let ((nn-eff (newest-non-effective object-designator)))
          (make-designator
           'object (description nn-eff) object-designator)))
      (cpl:fail 'cram-plan-failures:object-not-found
                :object-desig object-designator))
    results))

(def-action-handler examine (object-designator)
  (declare (ignorable object-designator))
  (ros-info (perception) "Examining object."))

(def-process-module robosherlock-process-module (desig)
  (apply #'call-action (reference desig)))
