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

(defun init-beliefstate ()
  (setf *registered-interactive-callbacks* nil))

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

(defun extract (name)
  (extract-files name))

(defun begin-experiment ()
  (let ((results (query-input `((experiment "Experiment name" "Pick and Place")
                                (robot "Robot" "PR2")
                                (creator "Creator" "Jan Winkler")))))
    (dolist (result results)
      (set-metadata (car result) (cdr result)))))

(defun end-experiment ()
  (let ((results (query-input `((:description "Description")))))
    (dolist (result results)
      (set-metadata (car result) (cdr result))))
  (let ((results (query-input `((:identifier "Identifier" "pick-and-place")))))
    (extract-files (find :identifier results :test (lambda (x y)
                                                     (eql x (car y)))))))

(defun query-input (data-fields)
  (mapcar (lambda (entry)
            (format t "~a" (second entry))
            (when (and (> (length entry) 2)
                       (not (string= (third entry) "")))
              (format t " (default '~a')" (third entry)))
            (format t ": ")
            (let* ((read-value (read-line))
                   (read-value (cond ((string= read-value "")
                                      (third entry))
                                     (t read-value))))
              (cons (first entry) read-value)))
          data-fields))
