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

(defparameter *red-color*
  (make-msg "std_msgs/ColorRGBA" :r 1.0 :g 0.0 :b 0.0 :a 0.7)
  "For internal use. ROS message representing the color red.")

(defparameter *collision-lifetime* 1.0
  "Lifetime of markers visualizing detected collisions.")

(defparameter *collision-marker-size* 0.2
  "Size of sphere visualizing detected collisions.")

(defun visualize-collisions (interface)
  "Visualizes collisions reported in feedback of beasty `interface' by publishing a red
 sperical marker at every joint which is in collision."
  (declare (type beasty-interface interface))
  ;; Note: If the user specified 'no-visualization' the publisher object is 'nil'.
  (when (visualization-pub interface)
    (let ((collision-markers 
            (create-collision-markers
             (joint-collisions (cram-language:value (state interface))))))
      ;; only publish if there is something to publish
      (when (> (length collision-markers) 0)
        (publish (visualization-pub interface)
                 (make-msg "visualization_msgs/MarkerArray" :markers collision-markers))))))

(defun create-collision-markers (collisions)
  "Returns vector of collision markers for joints which are in collision. 
 `collisions' is a vector expected to be joint collision feedback comming from
 Beasty."
  (declare (type vector collisions))
  (map 'vector #'make-collision-marker collisions))

(defun make-collision-marker (collision)
  "Creates a visualization marker-msg for the `collision' of type `collision'."
  (declare (type collision collision))
  (make-sphere-msg 
   (link-name collision) 
   *collision-marker-size*
   (get-joint-index (joint-name collision)) 
   *collision-lifetime*
   *red-color*))

(defun make-sphere-msg (frame-id radius id lifetime color)
  "Create a visualization marker for a sphere at `frame-id', with `radius' and `color'. 
 The marker will be shown for `lifetime' seconds and get `id' to discriminate it from
 other markers."
  (declare (type string frame-id)
           (type number radius id lifetime)
           (type std_msgs-msg:COLORRGBA color))
  (make-msg "visualization_msgs/Marker"
            :header (make-msg "std_msgs/Header" :frame_id frame-id :stamp (ros-time))
            :type 2 :action 0 :lifetime lifetime :id id :color color
            :scale (make-msg "geometry_msgs/Vector3" :x radius :y radius :z radius)))

(defun get-joint-index (joint-name)
  "Parses `joint-name' which is expected to match pattern *<index>* for the index of the
 joint and returns it as number."
  (declare (type string joint-name))
  (multiple-value-bind (joint-index string-position)
      (parse-integer (remove-if-not #'digit-char-p joint-name) :junk-allowed t)
    (declare (ignore string-position))
    joint-index))