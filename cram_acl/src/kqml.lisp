;;; Copyright (c) 2015, Jan Winkler <winkler@cs.uni-bremen.de>
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

(in-package :cram-acl)

;;;
;;; KQML Base Classes
;;;

(defclass kqml-base (acl-base)
  ((language :reader language :initarg :language)
   (ontology :reader ontology :initarg :ontology))
  (:default-initargs
   :acl-type 'kqml
   :language "KQML"
   :ontology nil))

(defclass kqml-communicatable (kqml-base)
  ((sender :reader sender :initarg :sender)
   (receiver :reader receiver :initarg :receiver))
  (:default-initargs
   :sender nil
   :receiver nil))

(defclass kqml-content-holder (kqml-base)
  ((content :reader content :initarg :content))
  (:default-initargs
   :content nil))

(defclass kqml-reaction (kqml-base)
  ((in-reply-to :reader in-reply-to :initarg :in-reply-to))
  (:default-initargs
   :in-reply-to nil))

;;;
;;; Basic Performatives
;;;

(defclass kqml-performative ()
  ((performative-type :reader performative-type :initarg :performative-type))
  (:default-initargs
   :performative-type nil))

(defclass kqml-performative-tell (kqml-performative
                                  kqml-communicatable
                                  kqml-content-holder
                                  kqml-reaction)
  ((force :reader force :initarg :force))
  (:default-initargs
   :force nil
   :performative-type 'tell))

(defclass kqml-performative-untell (kqml-performative-tell)
  ()
  (:default-initargs
   :performative-type 'untell))

(defclass kqml-performative-deny (kqml-performative
                                  kqml-communicatable
                                  kqml-content-holder
                                  kqml-reaction)
  ()
  (:default-initargs
   :performative-type 'deny))

;;;
;;; Basic Responses
;;;

(defclass kqml-response (kqml-reaction
                         kqml-communicatable)
  ((response-type :reader response-type :initarg :response-type)
   (comment :reader comment :initarg :comment))
  (:default-initargs
   :response-type nil
   :comment nil))

(defclass kqml-response-error (kqml-response)
  ((code :reader code :initarg :code))
  (:default-initargs
   :response-type 'error
   :code nil))

(defclass kqml-response-sorry (kqml-response)
  ()
  (:default-initargs
   :response-type 'sorry))

;;;
;;; Converter functions
;;;

(defgeneric kqml-tokens->hash-table (tokens))

(defmethod kqml-tokens->hash-table (tokens)
  (when (evenp (length tokens))
    (let ((hash-table (make-hash-table)))
      (loop for i from 0 below (length tokens) by 2
            for key = (elt tokens i)
            for value = (elt tokens (1+ i))
            do (setf (gethash (intern
                               (cond ((string= (subseq key 0 1) ":")
                                      (subseq key 1))
                                     (t key))
                               'acl)
                              hash-table)
                     value))
      hash-table)))

(defgeneric string->kqml (string))

(defmethod string->kqml (string)
  (let ((tokens (split-string string)))
    (when (> (length tokens) 0)
      (let ((first-token (first tokens))
            (kqml-hash-table (kqml-tokens->hash-table (rest tokens))))
        (labels ((hv (key)
                   (gethash (intern key 'acl) kqml-hash-table)))
          (cond ((string= first-token "tell")
                 (make-instance
                  'kqml-performative-tell
                  :content (hv "content")
                  :ontology (hv "ontology")
                  :in-reply-to (hv "in-reply-to")
                  :force (hv "force")
                  :sender (hv "sender")
                  :receiver (hv "receiver")))
                ((string= first-token "untell")
                 (make-instance
                  'kqml-performative-untell
                  :content (hv "content")
                  :ontology (hv "ontology")
                  :in-reply-to (hv "in-reply-to")
                  :force (hv "force")
                  :sender (hv "sender")
                  :receiver (hv "receiver")))
                ((string= first-token "deny")
                 (make-instance
                  'kqml-performative-deny
                  :content (hv "content")
                  :ontology (hv "ontology")
                  :in-reply-to (hv "in-reply-to")
                  :sender (hv "sender")
                  :receiver (hv "receiver")))
                ((string= first-token "error")
                 (make-instance
                  'kqml-response-error
                  :in-reply-to (hv "in-reply-to")
                  :sender (hv "sender")
                  :receiver (hv "receiver")
                  :comment (hv "comment")
                  :code (hv "code")))
                ((string= first-token "error")
                 (make-instance
                  'kqml-response-sorry
                  :in-reply-to (hv "in-reply-to")
                  :sender (hv "sender")
                  :receiver (hv "receiver")
                  :comment (hv "comment")))))))))

(defgeneric kqml-tokens->string (primary-expression kqml tokens))

(defmethod kqml-tokens->string (primary-expression kqml tokens)
  (reduce
   (lambda (str1 str2)
     (concatenate 'string
                  str1 (cond ((string= str1 "")
                              "")
                             (t " "))
                  str2))
   (append
    `(,primary-expression)
    (remove-if
     (lambda (string)
       (not string))
     (loop for token in tokens
           collect (when (and (slot-boundp kqml token)
                              (slot-value kqml token))
                     (concatenate
                      'string
                      ":" (string-downcase (symbol-name token)) " "
                      (slot-value kqml token)))
             into final-string
           finally (return final-string))))))

(defgeneric kqml->string (kqml))

(defmethod kqml->string ((kqml kqml-performative-tell))
  (kqml-tokens->string
   "tell" kqml
   `(content language ontology in-reply-to force sender receiver)))

(defmethod kqml->string ((kqml kqml-performative-untell))
  (kqml-tokens->string
   "untell" kqml
   `(content language ontology in-reply-to force sender receiver)))

(defmethod kqml->string ((kqml kqml-performative-deny))
  (kqml-tokens->string
   "deny" kqml
   `(content language ontology in-reply-to sender receiver)))

(defmethod kqml->string ((kqml kqml-response-error))
  (kqml-tokens->string
   "error" kqml
   `(in-reply-to sender receiver comment code)))

(defmethod kqml->string ((kqml kqml-response-sorry))
  (kqml-tokens->string
   "sorry" kqml
   `(in-reply-to sender receiver comment)))
