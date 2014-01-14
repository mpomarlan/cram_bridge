;;; Copyright (c) 2014, Georg Bartels <georg.bartels@cs.uni-bremen.de>
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

(in-package :cram-beasty)

(defparameter *collision-types* 
  (list :NO-COLLISION :CONTACT :LIGHT-COLLISION :STRONG-COLLISION :SEVERE-COLLISION)
  "List of symbols denoting all types of collisions beasty can report back.")

(defparameter *reaction-types*
  (list :IGNORE :ZERO-G :JOINT-IMP :SOFT-STOP :HARD-STOP)
  "List of symbols denoting all safety reactions beasty can exhibit.")

(defparameter *collision-index-map*
  (let ((collisions (make-hash-table)))
    (setf (gethash :CONTACT collisions) 0)
    (setf (gethash :LIGHT-COLLISION collisions) 1)
    (setf (gethash :STRONG-COLLISION collisions) 2)
    (setf (gethash :SEVERE-COLLISION collisions) 3)
    collisions)
  "Map to look up index of collision types (symbols) in beasty vector.")

(defparameter *collision-symbol-map*
  (let ((map (make-hash-table)))
    (loop for k being the hash-key in *collision-index-map* using (hash-value v) do
      (setf (gethash v map) k))
    map)
  "Map to look up collision types (symbols) given their index in a beasty vector.")

(defparameter *reaction-code-map*
  (let ((map (make-hash-table)))
    (setf (gethash :IGNORE map) 0)
    (setf (gethash :SOFT-STOP map) 1)
    (setf (gethash :HARD-STOP map) 2)
    (setf (gethash :JOINT-IMP map) 3)
    (setf (gethash :ZERO-G map) 4)
    map)
  "Map to lookup the beasty code for reaction strategies (symbols).")

(defparameter *default-thresholds*
  #(0.05 0.1 0.15 0.2 0.0 0.0 0.0 0.0)
  "Default collision detection thresholds for (in that order) :CONTACT, :LIGHT-COLLISION,
 :STRONG-COLLISION, :SEVERE-COLLISION, and 4x off. Values are in percent of maximum torque
 per joint.")

(defun make-safety-settings ()
  "Creates an empty instance of type 'safety-settings':"
  (make-instance 'safety-settings))

(defun has-collision-type-p (settings collision-type)
  "Checks whether `settings' holds a specification on how to react to `collision-type'."
  (declare (type safety-settings settings)
           (type symbol collision-type))
  (multiple-value-bind (value present) (gethash collision-type (strategies settings))
    (declare (ignore value))
    present))

(defun set-safety-strategy (settings collision-type reaction-type)
  "Sets the strategy to react to `collision-type' with `reaction-type' in beasty parameter
 `settings' which is of type 'safety-settings'."
  (declare (type safety-settings settings)
           (type symbol collision-type reaction-type))
  (setf (gethash collision-type (strategies settings)) reaction-type))

(defun remove-safety-strategy (settings collision-type)
  "Removes a specified reaction to `collision-type' from `settings'."
  (declare (type safety-settings settings)
           (type symbol collision-type))
  (when (has-collision-type-p settings collision-type)
    (remhash collision-type (strategies settings))))

(defun collision-type-valid-p (collision-type)
  "Checks whether the symbol `collision-type' denotes an exisiting type of collision."
  (declare (type symbol collision-type))
  (member collision-type *collision-types*))

(defun reaction-type-valid-p (reaction-type)
  "Checks whether the symbol `reaction-type' denotes an existing type of reaction."
  (declare (type symbol reaction-type))
  (member reaction-type *reaction-types*))

(defun strategy-valid-p (collision-type reaction-type)
  "Checks whether the symbols `collision-type' and `reaction-type' form a valid collision
 strategy specification."
  (declare (type symbol collision-type reaction-type))
  (and (collision-type-valid-p collision-type)
       (reaction-type-valid-p reaction-type)))

(defun all-strategies-valid-p (settings)
  "Checks whether all strategies defined in `settings' are valid."
  (declare (type safety-settings settings))
  (every 
   #'identity
   (loop for k being the hash-key in (strategies settings) using (hash-value v)
         collect (strategy-valid-p k v))))

(defun safety-settings-valid-p (settings)
  "Checks whether the beasty 'safety-settings' `settings' are valid, i.e. cover all possible
 collisions and use known reaction types."
  (declare (type safety-settings settings))
  (and (> 9 (hash-table-count (strategies settings)))
       (all-strategies-valid-p settings)))