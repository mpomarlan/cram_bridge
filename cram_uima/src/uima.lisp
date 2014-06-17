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
(defvar *uima-comm-mode* :topic)
(defvar *uima-service-topic* "/naive_perception")
(defvar *uima-trigger-service-topic* "/rs_collection_reader/trigger_uima_pipeline")
(defvar *uima-results-topic* "/rs_collection_reader/result_advertiser")

(define-condition uima-not-running () ())

(defun init-uima-bridge ()
  "Sets up the basic action client communication handles for the
UIMA framework."
  (config-uima)
  (setf *uima-result-fluent*
        (cpl:make-fluent
         :name 'uima-result
         :value nil
         :allow-tracing nil)))

(roslisp-utilities:register-ros-init-function init-uima-bridge)

(defun set-comm-mode (comm-mode
                      &key
                        service-topic
                        trigger-service-topic
                        results-topic)
  "Set the flag that decides how communication is taking place. The
default setting is to call the ROS service specified by
`service-topic' (`comm-mode' = `:service'). The only current
alternative is setting `comm-mode' to `:topic', which results in
sending a request message in a special UIMA trigger topic and waiting
for a reply on another topic."
  (setf *uima-service-topic* service-topic)
  (setf *uima-trigger-service-topic* trigger-service-topic)
  (setf *uima-results-topic* results-topic)
  (setf *uima-comm-mode* comm-mode)
  (when (eql comm-mode :topic)
    (when *uima-result-subscriber*
      ;;(roslisp:unsubscribe *uima-result-subscriber*)
      (setf *uima-result-subscriber* nil))
    (setf *uima-result-subscriber*
          (subscribe
           *uima-results-topic*
           "designator_integration_msgs/DesignatorResponse"
           #'uima-result-callback)))
  nil)

(defun uima-result-callback (msg)
  (cpl:setf (cpl:value *uima-result-fluent*) msg))

(defun trigger (designator-request)
  (cpl:with-failure-handling
      ((sb-bsd-sockets:connection-refused-error (f)
         (declare (ignore f))
         (cpl:retry)))
    (cpl:setf (cpl:value *uima-result-fluent*) nil)
    (desig-int::call-designator-service
     *uima-trigger-service-topic* designator-request)))
  ;; (roslisp:wait-for-service *uima-trigger-service-topic*)
  ;; (roslisp:call-service
  ;;  *uima-trigger-service-topic*
  ;;  'designator_integration_msgs-msg::DesignatorRequest
  ;;  :designator designator-request))

(defun make-perception-request-id ()
  (concatenate
   'string
   "request_"
   (write-to-string (random 1000000))
   "_"
   (write-to-string (random 1000000))))

(define-hook on-prepare-request (designator-request))
(define-hook on-finish-request (log-id result))

(defun get-uima-result (designator-request)
  (roslisp:ros-info (uima) "Waiting for perception results")
  (let ((log-id (first (on-prepare-request
                        designator-request)))
        (result-designators
          (cpl:with-failure-handling
              ((roslisp::ros-rpc-error (f)
                 (declare (ignore f))
                 (roslisp:ros-warn
                  (uima) "Waiting for connection to RoboSherlock.")
                 (sleep 1)
                 (cpl:retry)))
            (ecase *uima-comm-mode*
              (:topic
               ;;(trigger designator-request)
               (cpl:pursue
                 (cpl:sleep* 5) ;; Timeout
                 (when (cpl:wait-for *uima-result-fluent*)
                   (roslisp:with-fields (designators)
                       (cpl:value *uima-result-fluent*)
                     (map 'list (lambda (x)
                                  (desig-int::msg->designator x))
                          designators)))))
              (:service
               (desig-int::call-designator-service
                *uima-service-topic* designator-request))))))
    (roslisp:ros-info (uima) "Post processing perception results")
    (on-finish-request log-id result-designators)
    (progn
      (format t "Returning~%")
      result-designators)))

(defun config-uima ()
  (cram-uima:set-comm-mode
   :topic
   :trigger-service-topic *uima-trigger-service-topic*
   :results-topic *uima-results-topic*))

(defun config-naive-perception ()
  (cram-uima:set-comm-mode
   :service
   :service-topic "/naive_perception"))
