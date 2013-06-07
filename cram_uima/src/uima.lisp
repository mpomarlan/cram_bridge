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

(in-package :cram-uima)

(defvar *uima-client* nil)
(defvar *uima-result-subscriber* nil)
(defvar *uima-result-fluent* nil)
(defvar *uima-result-msg* nil)

(defun init-uima-bridge ()
  "Sets up the basic action client communication handles for the
UIMA framework."
  (setf *uima-result-subscriber*
        (subscribe
         "/uima/uima_result"
         "iai_msgs/PerceiveResult"
         #'uima-result-callback))
  (setf *uima-result-fluent*
        (cpl:pulsed (cpl:make-fluent
                     :name 'uima-result
                     :value nil
                     :allow-tracing nil))))

(register-ros-init-function init-uima-bridge)

(defun uima-result-callback (msg)
  (setf *uima-result-msg* msg)
  (cpl:pulse *uima-result-fluent*))

(defun trigger-uima ()
  (let ((service "/uima/trigger_uima_pipeline")
        (request-string "start"))
    (roslisp:wait-for-service service)
    (roslisp:call-service service
                          'iai_msgs-srv:TriggerUIMAPipeline
                          :str request-string)))

(defun get-uima-result ()
  (trigger-uima)
  (when (cpl:wait-for *uima-result-fluent* :timeout 5.0)
    (roslisp:with-fields (objects)
        *uima-result-msg*
      objects)))
