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

(defvar *planlogging-namespace* "/beliefstate_ros")
(defvar *kinect-topic-rgb* "/kinect_head/rgb/image_color")
(defvar *interactive-callback-fluent* (cpl:make-fluent))
(defparameter *interactive-callback-subscriber* nil)
(defparameter *registered-interactive-callbacks* nil)

(defun init-beliefstate ()
  (setf *registered-interactive-callbacks* nil)
  (setf *interactive-callback-subscriber*
        (roslisp:subscribe "/interactive_callback"
                           "designator_integration_msgs/Designator"
                           #'interactive-callback)))

(defun interactive-callback (msg)
  (let* ((desig (setf (cpl:value *interactive-callback-fluent*)
                      (designator-integration-lisp::msg->designator
                       msg)))
         (callback (cdr (find (desig-prop-value desig 'desig-props::command)
                              *registered-interactive-callbacks*
                              :test (lambda (x y)
                                      (string= x (car y)))))))
    (when callback
      (funcall callback (desig-prop-value desig 'desig-props::command)
                        (desig-prop-value desig 'desig-props::parameter)))))

(roslisp-utilities:register-ros-init-function init-beliefstate)

(defun fully-qualified-service-name (service-name)
  (concatenate 'string *planlogging-namespace* "/" service-name))

(defun start-node (name log-parameters detail-level)
  (let ((service (fully-qualified-service-name "begin_context")))
    (when (roslisp:wait-for-service service 0.3)
      (let* ((parameters
               (mapcar (lambda (x) x)
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
  (let ((service (fully-qualified-service-name "end_context")))
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
                                    (max-detail-level 99))
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

(defun extract-meta-file ()
  (extract-file "" 'meta))

(defun extract-file (filename format &key
                                       (successes t)
                                       (fails t)
                                       (max-detail-level 99))
  (ecase format
    (owl)
    (dot)
    (meta))
  (let ((service (fully-qualified-service-name "alter_context")))
    (when (roslisp:wait-for-service service 0.3)
      (designator-integration-lisp:call-designator-service
       service
       (cram-designators:make-designator
        'cram-designators:action
        (list (list 'command 'export-planlog)
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
  (let ((service (fully-qualified-service-name "control")))
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

(defun alter-node (designator)
  (let ((service (fully-qualified-service-name "alter_context")))
    (when (roslisp:wait-for-service service 0.3)
      (designator-integration-lisp:call-designator-service
       service
       designator))))

(defun start-new-experiment ()
  (let ((service (fully-qualified-service-name "alter_context")))
    (when (roslisp:wait-for-service service 0.3)
      (designator-integration-lisp:call-designator-service
       service
       (cram-designators:make-designator
        'cram-designators:action
        (list (list 'command 'start-new-experiment)))))))

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
      (desig-prop-value (first result) 'desig-props::id)))) ;; desig_id

(defun set-experiment-meta-data (field value)
  (alter-node
   (cram-designators:make-designator
    'cram-designators:action
    `((command set-experiment-meta-data)
      (field ,field)
      (value ,value)))))

(defun add-topic-image-to-active-node (image-topic)
  (alter-node
   (cram-designators:make-designator
    'cram-designators:action
    (list (list 'command 'add-image)
          (list 'origin image-topic)))))

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
      (let* ((desig-id (desig-prop-value (first result) 'desig-props::id)))
        desig-id))))

(defun set-metadata (&key robot creator experiment description)
  (when robot (set-experiment-meta-data "robot" robot))
  (when creator (set-experiment-meta-data "creator" creator))
  (when experiment (set-experiment-meta-data "experiment" experiment))
  (when description (set-experiment-meta-data "description" description)))

(defun equate-designators (desig-child desig-parent)
  (let* ((mem-addr-child (write-to-string
                          (sb-kernel:get-lisp-obj-address desig-child)))
         (mem-addr-parent (write-to-string
                           (sb-kernel:get-lisp-obj-address desig-parent)))
         (desc-child (description desig-child))
         (type-child (ecase (type-of desig-child)
                       (cram-designators:action-designator "ACTION")
                       (cram-designators:location-designator "LOCATION")
                       (cram-designators:object-designator "OBJECT")))
         (desc-parent (description desig-parent))
         (type-parent (ecase (type-of desig-parent)
                        (cram-designators:action-designator "ACTION")
                        (cram-designators:location-designator "LOCATION")
                        (cram-designators:object-designator "OBJECT"))))
    (alter-node
     (cram-designators:make-designator
      'cram-designators:action
      `((command equate-designators)
        (memory-address-child ,mem-addr-child)
        (type-child ,type-child)
        (description-child ,desc-child)
        (memory-address-parent ,mem-addr-parent)
        (type-parent ,type-parent)
        (description-parent ,desc-parent))))))

(defun extract-files (name)
  (let ((owl-name (concatenate 'string name ".owl"))
        (dot-name (concatenate 'string name ".dot"))
        (owl-name-no-details (concatenate 'string name "-no-details.owl"))
        (dot-name-no-details (concatenate 'string name "-no-details.dot")))
    (extract-dot-file dot-name)
    (extract-owl-file owl-name)
    (extract-meta-file)))
    ;(extract-dot-file dot-name-no-details :max-detail-level 2)
    ;(extract-owl-file owl-name-no-details :max-detail-level 2)))

(defun extract-mongodb-entries (&key
                                  (database "db_exp_log")
                                  (collection "col_exp_log"))
  (extract-mongodb database collection))

(defun extract (name)
  (extract-files name)
  (extract-mongodb-entries))

(defun register-interactive-object (name shape pose dimensions menu)
  (alter-node
   (cram-designators:make-designator
    'cram-designators:action
    (list (list 'command 'register-interactive-object)
          (list 'name name)
          (list 'shape shape)
          (list 'pose pose)
          (list 'width (elt dimensions 0))
          (list 'depth (elt dimensions 1))
          (list 'height (elt dimensions 2))
          (list 'menu menu)))))

(defun unregister-interactive-object (name)
  (alter-node
   (cram-designators:make-designator
    'cram-designators:action
    (list (list 'command 'unregister-interactive-object)
          (list 'name name)))))

(defun set-interactive-object-menu (name menu)
  (alter-node
   (cram-designators:make-designator
    'cram-designators:action
    (list (list 'command 'set-interactive-object-menu)
          (list 'name name)
          (list 'menu menu)))))

(defun set-interactive-object-pose (name pose)
  (alter-node
   (cram-designators:make-designator
    'cram-designators:action
    (list (list 'command 'set-interactive-object-pose)
          (list 'name name)
          (list 'pose pose)))))

(defun register-interactive-callback (command callback-function)
  "Callback functions need to receive two parameters: one for the
object name clicked, and one for the optional parameter given when the
menu entry for the interactive object was assigned."
  (setf *registered-interactive-callbacks*
        (remove command *registered-interactive-callbacks*
                :test (lambda (x y)
                        (string= x (car y)))))
  (push (cons command callback-function)
        *registered-interactive-callbacks*))

(defun unregister-interactive-callback (command callback-function)
  (setf *registered-interactive-callbacks*
        (remove command *registered-interactive-callbacks*
                :test (lambda (x y)
                        (and (string= x (car y))
                             (eql callback-function (cdr y)))))))
