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

(defvar *uima-node-name* "/RoboSherlock_common")

(defvar *uima-client* nil)
(defvar *uima-result-subscriber* nil)
(defvar *uima-result-fluent* nil)
(defvar *uima-result-msg* nil)
(defvar *uima-comm-mode* :topic)
(defvar *uima-service-topic* "/naive_perception")
(defvar *uima-trigger-service-topic*
  (concatenate 'string *uima-node-name* "/trigger_uima_pipeline"))
(defvar *uima-results-topic*
  (concatenate 'string *uima-node-name* "/result_advertiser"))

(defvar *stored-result* nil)

(defclass robosherlock-result ()
  ((content :reader content :initarg :content)
   (time-received :reader time-received :initarg :time-received)))

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
  (cpl:setf (cpl:value *uima-result-fluent*) msg)
  (setf *stored-result*
        (make-instance 'robosherlock-result
                       :content msg
                       :time-received (roslisp:ros-time))))

(defun trigger (designator-request)
  (cpl:with-failure-handling
      ((sb-bsd-sockets:connection-refused-error (f)
         (declare (ignore f))
         (cpl:retry)))
    (cpl:setf (cpl:value *uima-result-fluent*) nil)
    (desig-int::call-designator-service
     *uima-trigger-service-topic* designator-request)))

(define-hook cram-language::on-prepare-request (designator-request))
(define-hook cram-language::on-finish-request (log-id result))

(defun get-uima-result (designator-request &key (max-age 2.0))
  (let* ((call-result
           (cond ((and *stored-result*
                           (<= (- (roslisp:ros-time)
                                  (time-received *stored-result*))
                               max-age))
                      (roslisp:ros-info
                       (uima) "Found valid result in storage (~as old)."
                       (- (roslisp:ros-time) (time-received *stored-result*)))
                      (content *stored-result*))
                     (t
                      (roslisp:ros-info
                       (uima) "Waiting for perception results.")
                      (cpl:with-failure-handling
                          ((roslisp::ros-rpc-error (f)
                             (declare (ignore f))
                             (roslisp:ros-warn
                              (uima) "Waiting for connection to RoboSherlock.")
                             (sleep 1)
                             (cpl:retry)))
                        (ecase *uima-comm-mode*
                          (:topic
                           (cpl:pursue
                             (cpl:sleep* 5) ;; Timeout
                             (when (cpl:wait-for *uima-result-fluent*)
                               (cpl:value *uima-result-fluent*))))
                          (:service
                           (desig-int::call-designator-service
                            *uima-service-topic* designator-request)))))))
         (result-designators
           (when call-result
             (roslisp:with-fields (designators) call-result
               (map 'list (lambda (x)
                            (desig-int::msg->designator x))
                    designators)))))
    (unless call-result
      (roslisp:ros-error (uima) "No answer from UIMA. Is the node running?"))
    result-designators))

(defun config-uima ()
  (cram-uima:set-comm-mode
   :topic
   :trigger-service-topic *uima-trigger-service-topic*
   :results-topic *uima-results-topic*))
