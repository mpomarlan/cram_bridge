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

(defvar *planlogging-namespace* "/planlogger")
(defvar *designator-publisher* nil)
(defvar *designator-topic* "/logged_designators")

(defun beliefstate-init ()
  (setf *designator-publisher*
        (roslisp:advertise
         *designator-topic*
         'designator_integration_msgs-msg:designator)))

(register-ros-init-function beliefstate-init)

(defun fully-qualified-service-name (service-name)
  (concatenate 'string
               *planlogging-namespace*
               "/" service-name))

(defun start-node (name log-parameters detail-level)
  (let ((service (fully-qualified-service-name
                  "start_node")))
    (when (roslisp:wait-for-service service 0.3)
      (let* ((parameters
               (mapcar (lambda (x)
                         x)
                       (append (list (list '_name name)
                                     (list '_detail-level detail-level)
                                     (list '_source 'cram)
                                     log-parameters))))
             (result (first (designator-integration-lisp:call-designator-service
                             service
                             (cram-designators:make-designator
                              'cram-designators:action
                              parameters)))))
          (when result
            (desig-prop-value result 'desig-props::_id))))))

(defun stop-node (id &key (success t))
  (let ((service (fully-qualified-service-name
                  "stop_node")))
    (when (roslisp:wait-for-service service 0.3)
      (designator-integration-lisp:call-designator-service
       service
       (cram-designators:make-designator
        'cram-designators:action
        (list (list '_id id)
              (list '_success (cond (success 1)
                                    (t 0)))))))))

(defun extract-dot-file (filename &key
                                    (successes t)
                                    (fails t)
                                    (max-detail-level 99)
                                    (use-color t use-color-set-p))
  (when use-color-set-p
    (set-color-usage use-color))
  (extract-file filename 'dot
                :successes successes
                :fails fails
                :max-detail-level max-detail-level))

(defun extract-owl-file (filename &key
                                    (successes t)
                                    (fails t)
                                    (max-detail-level 99))
  (extract-file filename 'owl
                :successes successes
                :fails fails
                :max-detail-level max-detail-level))

(defun extract-file (filename format &key
                                       (successes t)
                                       (fails t)
                                       (max-detail-level 99))
  (ecase format
    (owl)
    (dot))
  (let ((service (fully-qualified-service-name
                  "control")))
    (when (roslisp:wait-for-service service 0.3)
      (designator-integration-lisp:call-designator-service
       service
       (cram-designators:make-designator
        'cram-designators:action
        (list (list 'command 'extract)
              (list 'format format)
              (list 'filename filename)
              (list 'show-successes (cond (successes 1)
                                          (t 0)))
              (list 'show-fails (cond (fails 1)
                                      (t 0)))
              (list 'max-detail-level max-detail-level)))))))

(defun extract-mongodb (database collection
                        &key
                          (successes t)
                          (fails t)
                          (max-detail-level 99))
  (let ((service (fully-qualified-service-name
                  "control")))
    (when (roslisp:wait-for-service service 0.3)
      (designator-integration-lisp:call-designator-service
       service
       (cram-designators:make-designator
        'cram-designators:action
        (list (list 'command 'extract)
              (list 'format 'mongo)
              (list 'database database)
              (list 'collection collection)
              (list 'show-successes (cond (successes 1)
                                          (t 0)))
              (list 'show-fails (cond (fails 1)
                                      (t 0)))
              (list 'max-detail-level max-detail-level)))))))

(defun set-color-usage (use-color)
  (let ((service (fully-qualified-service-name
                  "control")))
    (when (roslisp:wait-for-service service 0.3)
      (designator-integration-lisp:call-designator-service
       service
       (cram-designators:make-designator
        'cram-designators:action
        (list (list 'command 'use-color)
              (list 'use-color (cond (use-color 1)
                                     (t 0)))))))))

(defun alter-node (designator)
  (let ((service (fully-qualified-service-name
                  "alter_node")))
    (when (roslisp:wait-for-service service 0.3)
      (designator-integration-lisp:call-designator-service
       service
       designator))))

(defun add-object-to-active-node (designator &key (annotation ""))
  (let* ((memory-address (write-to-string
                          (sb-kernel:get-lisp-obj-address designator)))
         (description (description designator))
         (result (alter-node
                  (cram-designators:make-designator
                   'cram-designators:action
                   (list (list 'command 'add-object)
                         (list 'type "OBJECT")
                         (list 'memory-address memory-address)
                         (list 'description description))))))
    (add-designator-to-active-node designator :annotation annotation)
    (when result
      (let* ((desig-id (desig-prop-value (first result) 'desig-props::id))
             (is-new (eql (desig-prop-value (first result) 'desig-props::is-new)
                          1.0d0)))
        (when is-new
          (publish-logged-designator
           (type-of designator)
           (append description (list `(__id ,desig-id)))))
        desig-id))))

(defun add-topic-image-to-active-node (image-topic)
  (let ((filename "topic.png"))
    (alter-node
     (cram-designators:make-designator
      'cram-designators:action
      (list (list 'command 'add-image)
            (list 'origin image-topic)
            (list 'filename filename))))))

(defun add-failure-to-active-node (condition)
  (let ((cond-str (write-to-string condition)))
    (alter-node
     (cram-designators:make-designator
      'cram-designators:action
      (list (list 'command 'add-failure)
            (list 'condition cond-str))))))

(defun add-designator-to-active-node (designator &key (annotation ""))
  (let* ((type (ecase (type-of designator)
                 (cram-designators:action-designator "ACTION")
                 (cram-designators:location-designator "LOCATION")
                 (cram-designators:object-designator "OBJECT")))
         (memory-address (write-to-string
                          (sb-kernel:get-lisp-obj-address designator)))
         (description (description designator))
         (result (alter-node
                  (cram-designators:make-designator
                   'cram-designators:action
                   (list (list 'command 'add-designator)
                         (list 'type type)
                         (list 'annotation annotation)
                         (list 'memory-address memory-address)
                         (list 'description description))))))
    (when result
      (let* ((desig-id (desig-prop-value (first result) 'desig-props::id))
             (is-new (eql (desig-prop-value (first result) 'desig-props::is-new)
                          1.0d0)))
        (when is-new
          (publish-logged-designator
           (type-of designator)
           (append description (list `(__id ,desig-id)))))
        desig-id))))

(defun publish-logged-designator (type description)
  (roslisp:publish
   *designator-publisher*
   (designator-integration-lisp::designator->msg
    (make-designator (ecase type
                       (cram-designators:action-designator
                        'cram-designators:action)
                       (cram-designators:location-designator
                        'cram-designators:location)
                       (cram-designators:object-designator
                        'cram-designators:object))
                     description))))

(defun equate-designators (desig-child desig-parent)
  (let* ((mem-addr-child (write-to-string
                          (sb-kernel:get-lisp-obj-address desig-child)))
         (mem-addr-parent (write-to-string
                           (sb-kernel:get-lisp-obj-address desig-parent))))
    (let ((result
            (first (alter-node
                    (cram-designators:make-designator
                     'cram-designators:action
                     (list (list 'command 'equate-designators)
                           (list 'memory-address-child mem-addr-child)
                           (list 'memory-address-parent mem-addr-parent)))))))
      (when result
        (let* ((desig-id-child (desig-prop-value result
                                                 'desig-props::id-child))
               (is-new-child (eql (desig-prop-value
                                   result
                                   'desig-props::is-new-child)
                                  1.0d0))
               (desig-id-parent (desig-prop-value result
                                                  'desig-props::id-parent))
               (is-new-parent (eql (desig-prop-value
                                    result
                                    'desig-props::is-new-parent)
                                   1.0d0)))
          (when is-new-child
            (publish-logged-designator
             (type-of desig-child)
             (append (description desig-child)
                     (list `(__id ,desig-id-child)))))
          (when is-new-parent
            (publish-logged-designator
             (type-of desig-parent)
             (append (description desig-parent)
                     (list `(__id ,desig-id-parent))))))))))

(defun extract-files (name)
  (let ((owl-name (concatenate 'string name ".owl"))
        (dot-name (concatenate 'string name ".dot"))
        (owl-name-no-details (concatenate 'string name "-no-details.owl"))
        (dot-name-no-details (concatenate 'string name "-no-details.dot")))
    (extract-dot-file dot-name)
    (extract-owl-file owl-name)
    (extract-dot-file dot-name-no-details :max-detail-level 2)
    (extract-owl-file owl-name-no-details :max-detail-level 2)))

(defun extract-mongodb-entries (&key
                                  (database "db_exp_log")
                                  (collection "col_exp_log"))
  (extract-mongodb database collection))

(defun extract (name)
  (extract-files name)
  (extract-mongodb-entries))
