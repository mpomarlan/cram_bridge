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
;;;     * Neither the name of the Universitaet Bremen nor the names of its contributors 
;;;       may be used to endorse or promote products derived from this software 
;;;       without specific prior written permission.
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

(in-package :cram-beliefstate)

(defvar *enable-prediction* nil)

(defun annotate-parameters (parameters)
  (add-designator-to-active-node
   (make-designator 'object `(,parameters))
   :annotation "parameter-annotation"))

(defun annotate-parameter (symbol value)
  (annotate-parameters `((,symbol ,value))))

(defun predict (parameters)
  "Predict the outcome of the current branch."
  (designator-integration-lisp:call-designator-service
   "/beliefstate_ros/predict"
   (make-designator 'action parameters)))

(defun call-predict (features-hashes)
  (let ((features (loop for h being the hash-keys of features-hashes
                        collect `(,h ,(gethash h features-hashes)))))
    (let* ((prediction (first (predict features)))
           (failures-returned (desig-prop-value prediction 'desig-props::failures)))
      (cond ((listp failures-returned)
             (mapcar (lambda (failure-returned)
                       `(,(intern (symbol-name
                                   (car failure-returned))
                                  'cram-plan-failures)
                         ,(cdr failure-returned)))
                     failures-returned))
            (t nil)))))

(defun predict-failures-hash-table (features-hash-table)
  (format t "Predicting all possible failures~%")
  (let ((hashes (make-hash-table)))
    hashes))

(defun predict-values-hash-table (keys features-hash-table)
  (format t "Predicting values with keys: ~a~%" keys)
  (let ((hashes (make-hash-table)))
    (dolist (key keys)
      (setf (gethash key hashes) nil)
      (case key
        (time (setf (gethash key hashes) 12.7))))
    hashes))

(defun annotate-features (features-hash-table)
  (loop for h being the hash-keys of features-hash-table
        as v = (gethash h features-hash-table)
        do (annotate-parameter h v)
           (format t "Annotated feature: ~a = ~a~%" h v)))

;; (defmacro choose (tag
;;                   &key generators features
;;                     constraints predicting (attempts 1)
;;                     body)
;;   `(block choose-block
;;      (let ((generated-param-hash-table (make-hash-table))
;;            (attempts ,attempts))
;;        (labels ((generate-parameters ()
;;                   ,@(loop for (variables generator) in generators
;;                           collect
;;                           `(let ((generated-values ,generator))
;;                              ,(loop for i from 0 below (length variables)
;;                                     as variable = (nth i variables)
;;                                     append `(setf (gethash
;;                                                    ',variable
;;                                                    generated-param-hash-table)
;;                                                   (nth ,i generated-values)))))))
;;          (loop with continue = t
;;                while (and continue (>= (decf attempts) 0))
;;                do (progn
;;                     (generate-parameters)
;;                     (let* ,(mapcar (lambda (parameter)
;;                                      `(,parameter (gethash
;;                                                    ',parameter
;;                                                    generated-param-hash-table)))
;;                             parameters)
;;                       (let* ,(append
;;                               `((feature-hashes (make-hash-table)))
;;                               (mapcar (lambda (feature)
;;                                         (destructuring-bind (var gen)
;;                                             feature
;;                                           `(,var (setf (gethash
;;                                                         ',var feature-hashes)
;;                                                        ,gen))))
;;                                       features))
;;                         (let ((predicted-failures
;;                                 (when *enable-prediction*
;;                                   (predict-failures-hash-table
;;                                    feature-hashes))))
;;                           (when (or (not *enable-prediction*)
;;                                     (and ,@(mapcar
;;                                             (lambda (constraint)
;;                                               (destructuring-bind (failure
;;                                                                    comparator)
;;                                                   constraint
;;                                                 `(let ((predicted-failure
;;                                                          (gethash
;;                                                           ',failure
;;                                                           predicted-failures)))
;;                                                    ,comparator)))
;;                                             constraints)))
;;                             (setf continue nil)
;;                             (annotate-features feature-hashes)
;;                             (let ((predicted-values (predict-values-hash-table
;;                                                      ',predicting
;;                                                      feature-hashes)))
;;                               (labels ((predicted (key)
;;                                          (gethash key predicted-values)))
;;                                 (return-from choose-block
;;                                   (values (progn ,body) t)))))))))))
;;        (return-from choose-block (values nil nil)))))
