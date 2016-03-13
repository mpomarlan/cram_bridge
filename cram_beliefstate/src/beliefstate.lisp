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

(defvar *planlogging-namespace* "/semrec_ros")
(defvar *kinect-topic-rgb* "/kinect_head/rgb/image_color")
(defvar *interactive-callback-fluent* (cpl:make-fluent))
(defvar *service-access* (make-lock :name "logging-service-access-lock"))

(defparameter *logging-enabled* t)
(defparameter *interactive-callback-subscriber* nil)
(defparameter *registered-interactive-callbacks* nil)

(defgeneric alter-node (description &key node-id mode command))

(defun init-semrec ()
  (setf *registered-interactive-callbacks* nil))

(roslisp-utilities:register-ros-init-function init-semrec)

(defun change-planlogging-namespace(&key (namespace "semrec_ros") (use-roslisp-ns nil))
  (if use-roslisp-ns
      (setq *planlogging-namespace* (concatenate 'string roslisp::*namespace* (string-left-trim "/" namespace)))
      (setq *planlogging-namespace* (concatenate 'string "/" (string-left-trim "/" namespace)))))
      

(defun fully-qualified-service-name (service-name)
  (concatenate 'string *planlogging-namespace* "/" service-name))

(defun wait-for-logging (service &optional (duration 0.3))
  (when *logging-enabled*
    (let* ((full-service-name (fully-qualified-service-name service))
           (result (roslisp:wait-for-service full-service-name duration)))
      (unless result
        (setf *logging-enabled* nil)
        (roslisp:ros-warn
         (semrec)
         "No connection to semrec service '~a'. Disabling logging."
         service))
      result)))

(defun logging-enabled ()
  *logging-enabled*)

(defun toggle-logging ()
  (if *logging-enabled*
    (roslisp:ros-info (semrec) "Switching OFF semrec logging.")
    (roslisp:ros-info (semrec) "Switching ON semrec logging."))
  (setf *logging-enabled* (not *logging-enabled*)))

(defun enable-logging (bool)
  (setf *logging-enabled* bool))

(defun start-node (name &optional log-parameters (detail-level 2) log-id add-params)
  (with-lock-held (*service-access*)
    (when (wait-for-logging "operate")
      (let* ((parameters
               (remove-if-not
                #'identity
                (append (list (list '_cb_type 'begin)
                              (list '_name name)
                              (list '_detail-level detail-level)
                              (list '_source 'cram)
                              log-parameters)
                        (when log-id
                          `((_relative_context_id ,log-id)))
                        add-params)))
             (result (first (designator-integration-lisp:call-designator-service
                             (fully-qualified-service-name "operate")
                             (cram-designators:make-designator
                              'cram-designators:action
                              parameters)))))
        (when result
          (desig-prop-value result 'desig-props::_id))))))
  

(defun stop-node (id &key (success t) relative-context-id)
  (with-lock-held (*service-access*)
    (when (wait-for-logging "operate")
      (let ((params
              (remove-if-not
               #'identity
               (append
                (list (list '_cb_type 'end)
                      (list '_id id)
                      (list '_success (cond (success 1)
                                            (t 0))))
                (when relative-context-id
                  (list (list '_relative_context_id relative-context-id)))))))
        (designator-integration-lisp:call-designator-service
         (fully-qualified-service-name "operate")
         (cram-designators:make-designator
          'cram-designators:action
          params))))))

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
  (alter-node
   (list (list 'command 'export-planlog)
         (list 'format format)
         (list 'filename filename)
         (list 'show-successes (cond (successes 1)
                                     (t 0)))
         (list 'show-fails (cond (fails 1)
                                 (t 0)))
         (list 'max-detail-level max-detail-level))))

(defmethod alter-node ((description list) &key node-id (mode :alter) (command "")
                                            (assurance-token ""))
  (with-lock-held (*service-access*)
    (when (wait-for-logging "operate")
      (let ((description (cond (node-id (append description `((_relative_context_id ,node-id))))
                               (t description))))
        (designator-integration-lisp:call-designator-service
         (fully-qualified-service-name "operate")
         (make-designator 'action
                          (append
                           description
                           (cond ((eql mode :service)
                                  `((command ,command))))
                           (cond ((not (string= assurance-token ""))
                                  `((_assurance_token ,assurance-token))))
                           `((_cb_type alter))
                           `((_type ,(case mode
                                       (:alter "alter")
                                       (:service "service")))))))))))

(defun start-new-experiment ()
  (let ((service (fully-qualified-service-name "alter_context")))
    (when (roslisp:wait-for-service service 0.3)
      (designator-integration-lisp:call-designator-service
       service
       (cram-designators:make-designator
        'cram-designators:action
        (list (list 'command 'start-new-experiment)))))))

(defun add-object (designator &key (path-to-cad-model "") (annotation ""))
  (let* ((memory-address (write-to-string
                          (sb-kernel:get-lisp-obj-address designator)))
         (description (description designator)))
    (alter-node
     (list (list 'command 'add-object)
           (list 'type "OBJECT")
           (list 'path-to-cad-model path-to-cad-model)
           (list 'annotation annotation)
           (list 'memory-address memory-address)
           (list 'description description)))))

(defun add-object-to-active-node (designator &key (path-to-cad-model "") (annotation ""))
  (let ((result (add-object
                 designator
                 :path-to-cad-model path-to-cad-model
                 :annotation annotation)))
    (when result
      (add-designator-to-active-node
       result
       :annotation annotation)
      (desig-prop-value (first result) 'desig-props::id))))

(defun add-object-to-node (designator id &key (path-to-cad-model "") (annotation ""))
  (let ((result (add-object
                 designator
                 :path-to-cad-model path-to-cad-model
                 :annotation annotation)))
    (when result
      (add-designator-to-node designator id :annotation annotation)
      (desig-prop-value (first result) 'desig-props::id))))

(defun add-human (designator &key (tf-prefix "") (srdl-component "") (property "")
                               (relative-context-id))
  (let* ((memory-address (write-to-string
                          (sb-kernel:get-lisp-obj-address designator)))
         (description (description designator)))
    (alter-node
     (remove-if-not
      #'identity
      (list (list 'command 'add-human)
            (list 'type "HUMAN")
            (list 'tf-prefix tf-prefix)
            (list 'srdl-component srdl-component)
            (list 'property property)
            (list 'memory-address memory-address)
            (list 'description description)
            (when relative-context-id
              (list '_relative_context_id relative-context-id)))))))

(defun add-human-to-node (designator id &key (tf-prefix "") (srdl-component "") (property "")
                                     (relative-context-id))
  (let ((result (add-human
                 designator
                 :tf-prefix tf-prefix
                 :srdl-component srdl-component
                 :property property
                 :relative-context-id relative-context-id)))
    (when result
      (add-designator-to-node
       designator id :annotation property
                     :relative-context-id relative-context-id)
      (desig-prop-value (first result) 'desig-props::id))))

(defun catch-current-failure-with-active-node (id)
  (alter-node (list (list 'command 'catch-failure)
                    (list 'context-id id))))

(defun rethrow-current-failure-with-active-node (id)
  (alter-node (list (list 'command 'rethrow-failure)
                    (list 'context-id id))))

(defun set-experiment-meta-data (field value)
  (alter-node
   `((command set-experiment-meta-data)
     (field ,field)
     (value ,value))))

(defun add-topic-image-to-active-node (image-topic)
  (alter-node
   (list (list 'command 'add-image)
         (list 'origin image-topic))))

(defun add-failure-to-active-node (datum)
  (let ((datum-str (cond (datum (write-to-string datum))
                        (t "ANONYMOUS-FAILURE"))))
    (alter-node
     (list (list 'command 'add-failure)
           (list 'condition datum-str)))))

(defun add-designator-to-node (designator node-id &key (annotation "") (relative-context-id))
  (let* ((type (ecase (type-of designator)
                 (cram-designators:action-designator "ACTION")
                 (cram-designators:location-designator "LOCATION")
                 (cram-designators:human-designator "HUMAN")
                 (cram-designators:object-designator "OBJECT")))
         (memory-address (write-to-string
                          (sb-kernel:get-lisp-obj-address designator)))
         (description (description designator))
         (result (alter-node
                  (remove-if-not
                   #'identity
                   (list (list 'command 'add-designator)
                         (list 'type type)
                         (list 'annotation annotation)
                         (list 'memory-address memory-address)
                         (list 'description description)
                         (when relative-context-id
                           (list '_relative_context_id relative-context-id))))
                  :node-id node-id)))
    (when result
      (let* ((desig-id (desig-prop-value (first result) 'desig-props::id)))
        desig-id))))

(defun resolve-designator-memory-address (designator)
  (let ((result
          (alter-node `((memory-address
                         ,(write-to-string
                           (sb-kernel:get-lisp-obj-address
                            designator))))
                      :mode :service
                      :command
                      "resolve-designator-memory-address")))
    (when result
      (desig-prop-value (first result) 'desig-props::id))))

(defun resolve-designator-knowrob-live-id (designator-id)
  (let ((result
          (alter-node `((designator-id ,designator-id))
                      :mode :service
                      :command
                      "resolve-designator-knowrob-live-id")))
    (when result
      (desig-prop-value (first result)
                        'desig-props::knowrob-live-id))))

(defun resolve-designator-knowrob-id (designator)
  "Resolves the ID given to designators when asserted into a live KnowRob instance."
  (resolve-designator-knowrob-live-id
   (resolve-designator-memory-address designator)))

(defun add-designator-to-active-node (designator &key (annotation "")
                                                   property-namespace)
  (let* ((type (ecase (type-of designator)
                 (cram-designators:action-designator "ACTION")
                 (cram-designators:location-designator "LOCATION")
                 (cram-designators:human-designator "HUMAN")
                 (cram-designators:object-designator "OBJECT")))
         (memory-address (write-to-string
                          (sb-kernel:get-lisp-obj-address designator)))
         (description (description designator))
         (result (alter-node
                  (append
                   (list (list 'command 'add-designator)
                         (list 'type type)
                         (list 'annotation annotation)
                         (list 'memory-address memory-address)
                         (list 'description description))
                   (when property-namespace
                     (list (list 'namespace property-namespace)))))))
    (when result
      (let* ((desig-id (desig-prop-value (first result) 'desig-props::id)))
        desig-id))))

(defun set-metadata (&key (robot "PR2") (creator "IAI") experiment description (cram-semrec-version "0.4"))
  (when robot (set-experiment-meta-data "robot" robot))
  (when creator (set-experiment-meta-data "creator" creator))
  (when experiment (set-experiment-meta-data "experiment" experiment))
  (when description (set-experiment-meta-data "description" description))
  (when cram-semrec-version (set-experiment-meta-data "cram-semrec-version" cram-semrec-version)))

(defun equate-designators (desig-child desig-parent)
  (let* ((mem-addr-child (write-to-string
                          (sb-kernel:get-lisp-obj-address desig-child)))
         (mem-addr-parent (write-to-string
                           (sb-kernel:get-lisp-obj-address desig-parent)))
         (desc-child (description desig-child))
         (type-child (ecase (type-of desig-child)
                       (cram-designators:action-designator "ACTION")
                       (cram-designators:location-designator "LOCATION")
                       (cram-designators:human-designator "HUMAN")
                       (cram-designators:object-designator "OBJECT")))
         (desc-parent (description desig-parent))
         (type-parent (ecase (type-of desig-parent)
                        (cram-designators:action-designator "ACTION")
                        (cram-designators:location-designator "LOCATION")
                        (cram-designators:human-designator "HUMAN")
                        (cram-designators:object-designator "OBJECT"))))
    (alter-node
     `((command equate-designators)
       (memory-address-child ,mem-addr-child)
       (type-child ,type-child)
       (description-child ,desc-child)
       (memory-address-parent ,mem-addr-parent)
       (type-parent ,type-parent)
       (description-parent ,desc-parent)))))

(defun extract-files (&key (name "cram_log") detail-level)
  (extract-meta-file)
  (unless detail-level
    (let ((owl-name (concatenate 'string name ".owl"))
          (dot-name (concatenate 'string name ".dot")))
      (extract-dot-file dot-name)
      (extract-owl-file owl-name)))
  (when detail-level
    (let ((owl-name-low-details (concatenate 'string name "_low_details.owl"))
          (dot-name-low-details (concatenate 'string name "_low_details.dot")))
      (extract-dot-file dot-name-low-details :max-detail-level detail-level)
      (extract-owl-file owl-name-low-details :max-detail-level detail-level))))

(defun extract (name)
  (extract-files :name name))

(defun begin-experiment ()
  (let ((results (query-input `((experiment "Experiment name" "Pick and Place")
                                (robot "Robot" "PR2")
                                (creator "Creator" "IAI")))))
    (dolist (result results)
      (set-metadata (car result) (cdr result)))))

(defun end-experiment ()
  (let ((results (query-input `((:description "Description")))))
    (dolist (result results)
      (set-metadata (car result) (cdr result))))
  (let ((results (query-input `((:identifier "Identifier" "pick-and-place")))))
    (extract-files :name (find :identifier results :test (lambda (x y)
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

(defun register-owl-namespace (shortcut iri relative-context-id)
  (alter-node (list (list 'command "register-owl-namespace")
                    (list 'shortcut shortcut)
                    (list 'iri iri))
              :node-id relative-context-id))
