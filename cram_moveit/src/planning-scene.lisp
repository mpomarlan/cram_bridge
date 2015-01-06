;;; Copyright (c) 2014, Jan Winkler <winkler@cs.uni-bremen.de>
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

(in-package :cram-moveit)

(defclass collision-matrix ()
  ((names :reader names :initarg :names)
   (entries :reader entries :initarg :entries)))

(defgeneric relative-collision-matrix (names-groups-1 names-groups-2 values &key matrix))
(defgeneric relative-collision-matrix-msg (names-groups-1 names-groups-2 values &key matrix))

(defun get-planning-scene (components)
  (call-service "/get_planning_scene"
                'moveit_msgs-srv:GetPlanningScene
                :components
                (make-message "moveit_msgs/PlanningSceneComponents"
                              :components components)))

(defun get-allowed-collision-matrix ()
  (get-planning-scene
   (roslisp-msg-protocol:symbol-code
    'moveit_msgs-msg:planningscenecomponents
    :allowed_collision_matrix)))

(defun msg->collision-matrix (msg)
  (with-fields (scene) msg
    (with-fields (allowed_collision_matrix) scene
      (with-fields (entry_names entry_values) allowed_collision_matrix
        (make-instance
         'collision-matrix
         :names (map 'list #'identity entry_names)
         :entries (make-array
                   `(,(length entry_values) ,(length entry_values))
                   :initial-contents
                   (map 'vector
                        (lambda (entry-line)
                          (with-fields (enabled) entry-line
                            enabled))
                        entry_values)))))))

(defun get-collision-matrix-entry (matrix name-1 name-2)
  (let ((idx-1 (position name-1 (names matrix) :test #'string=))
        (idx-2 (position name-2 (names matrix) :test #'string=)))
    (when (and idx-1 idx-2)
      (aref (entries matrix) idx-1 idx-2))))

(defun set-collision-matrix-entry (matrix name-1 name-2 value)
  (let* ((old-names (names matrix))
         (new-names (remove-duplicates (append old-names `(,name-1 ,name-2))
                                       :test #'string=)))
    (make-instance
     'collision-matrix
     :names new-names
     :entries
     (make-array
      `(,(length new-names) ,(length new-names))
      :initial-contents
      (map 'vector
           (lambda (ref-name-1)
             (map 'vector
                  (lambda (ref-name-2)
                    (cond ((or (and (string= ref-name-1 name-1)
                                    (string= ref-name-2 name-2))
                               (and (string= ref-name-1 name-2)
                                    (string= ref-name-2 name-1)))
                           value)
                          (t (get-collision-matrix-entry
                              matrix ref-name-1 ref-name-2))))
                  new-names))
           new-names)))))

(defun collision-matrix->msg (matrix)
  (make-message
   "moveit_msgs/AllowedCollisionMatrix"
   :entry_names (map 'vector #'identity (names matrix))
   :entry_values
   (let* ((entries (entries matrix))
          (dimensions (array-dimensions entries)))
     (map 'vector #'identity
          (loop for i from 0 below (elt dimensions 0)
                collecting
                (make-message
                 "moveit_msgs/AllowedCollisionEntry"
                 :enabled
                 (map 'vector #'identity
                      (loop for j from 0 below (elt dimensions 1)
                            collecting (aref entries i j)))))))))

(defmethod relative-collision-matrix (names-groups-1 names-groups-2 values &key matrix)
  (cond (matrix
         (cond ((> (length names-groups-1) 0)
                (let ((new-matrix
                        (set-collision-matrix-entry
                         matrix
                         (first names-groups-1) (first names-groups-2) (first values))))
                  (relative-collision-matrix
                   (rest names-groups-1) (rest names-groups-2) (rest values)
                   :matrix new-matrix)))
               (t matrix)))
        (t (let* ((expanded-groups
                    (loop for i from 0 below (length names-groups-1)
                          appending
                          (loop for name-group-1 in (elt names-groups-1 i)
                                appending
                                (loop for name-group-2 in (elt names-groups-2 i)
                                      collecting
                                      `(,name-group-1 ,name-group-2 ,(elt values i))))))
                  (names-1 (mapcar (lambda (group) (first group)) expanded-groups))
                  (names-2 (mapcar (lambda (group) (second group)) expanded-groups))
                  (values (mapcar (lambda (group) (third group)) expanded-groups)))
             (relative-collision-matrix
              names-1 names-2 values
              :matrix (msg->collision-matrix
                       (get-allowed-collision-matrix)))))))

(defmethod relative-collision-matrix-msg (names-groups-1 names-groups-2 values &key matrix)
  (declare (ignore matrix))
  (collision-matrix->msg
   (combine-collision-matrices `(,@(generate-collision-matrices
                                    names-groups-1 names-groups-2 values)
                                 ,(msg->collision-matrix (get-allowed-collision-matrix))))))

(defun generate-collision-matrices (names-groups-1 names-groups-2 values)
  (mapcar (lambda (names-1 names-2 value)
            (let ((all-names (remove-duplicates
                              (append names-1 names-2)
                              :test #'string=)))
              (make-instance
               'collision-matrix
               :names all-names
               :entries
               (make-array
                `(,(length all-names) ,(length all-names))
                :initial-contents
                (map
                 'vector (lambda (name-1)
                           (map
                            'vector (lambda (name-2)
                                      (cond ((or (and (find name-1 names-1 :test #'string=)
                                                      (find name-2 names-2 :test #'string=))
                                                 (and (find name-1 names-2 :test #'string=)
                                                      (find name-2 names-1 :test #'string=)))
                                             value)
                                            (t :maybe)))
                            all-names))
                     all-names)))))
          names-groups-1 names-groups-2 values))

(defun combine-collision-matrices (matrices)
  (let ((all-names
          (remove-duplicates
           (loop for matrix in matrices
                 appending (names matrix))
           :test #'string=)))
    (make-instance
     'collision-matrix
     :names all-names
     :entries
     (make-array
      `(,(length all-names) ,(length all-names))
      :initial-contents
      (map
       'vector #'identity
       (loop for name-1 in all-names
             collecting
             (map
              'vector #'identity
              (loop for name-2 in all-names
                    collecting
                    (cond ((string= name-1 name-2) t)
                          (t (block check
                               (loop for matrix in matrices
                                     for names = (names matrix)
                                     for present = (and (find name-1 names :test #'string=)
                                                        (find name-2 names :test #'string=))
                                     when present
                                       do (let ((value (get-collision-matrix-entry
                                                        matrix name-1 name-2)))
                                            (when (not (eql value :maybe))
                                              (return-from check value)))))))))))))))
