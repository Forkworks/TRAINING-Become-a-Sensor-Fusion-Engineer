# TRAINING-Become-a-Sensor-Fusion-Engineer


![uber_setup](https://user-images.githubusercontent.com/85563304/122566204-abce8700-d01d-11eb-9146-ed866362bf0e.jpg)

Uber Autonomous Setup


###### Sensors Available with their respectful advantages and disadvantages

Let us take a look at those sensor classes one-by one:

**Cameras:** Ubers fleet of modified Volvo XC90 SUVs features a range of cameras on their roofs, plus additional cameras that point to the sides and behind the car. The roof cameras are able to focus both close and far field, watch for braking vehicles, crossing pedestrians, traffic lights, and signage. The cameras feed their material to a central on-board computer, which also receives the signals of the other sensors to create a precise image of the vehicle’s surroundings. Much like human eyes, the performance of camera systems at night is strongly reduced, which makes them less reliable to locate objects with the required detection rates and positional accuracy. This is why the Uber fleet is equipped with two additional sensor types.

**Radar:** Radar systems emit radio waves which are reflected off of (many but not all) objects. The returning waves can be analyzed with regard to their runtime (which gives distance) as well as their shifted frequency (which gives relative speed). The latter property clearly distinguished the radar from the other two sensor types as it is the only one who is able to directly measure the speed of objects. **Also, radar is very robust against adverse weather conditions like heavy snow and thick fog.** Used by cruise control systems for many years, radar works best when identifying larger objects with good reflective properties. When it comes to detecting smaller or „soft“ objects (humans, animals) with reduced reflective properties, the radar detection performance drops. Even though camera and radar combine well, there are situations where both sensors do not work optimally - which is why Uber chose to throw a third sensor into the mix.

**Lidar :** Lidar works in a similar way to radar, but instead of emitting radio waves it uses infrared light. The roof-mouted sensor rotates at a high velocity and builds a detailed 3D image of its surroundings. In case of the Velodyne VLS-128, a total of 128 laser beams is used to detect obstacles up to a distance of 300 meters. During a single spin through 360 degrees, a total of up to 4 million datapoints per second is generated. Similar to the camera, Lidar is an optical sensor. It has the significant advantage however of "bringing its own light source“, whereas cameras are dependent on ambient light and the vehicle headlights. It has to be noted however, that Lidar performance is also reduced in adverse environmental conditions such as snow, heavy rain or fog. Coupled with low reflective properties of certain materials, a Lidar might thus fail at generating a sufficiently dense point cloud for some objects in traffic, leaving only a few 3D points with which to work. It is thus a good idea to combine Lidar with other sensors to ensure that detection performance is sufficiently high for autonomous navigation through traffic.


Sensor | Range measurement | Robustness in darkness | Robustness in rain, snow, or fog | Classification of objects | Perceiving 2D Structures | Measure speed / TTC | Package size
--- | --- | --- | --- |--- |--- |--- |---
Camera | - | - | - | ++ | ++ | + | + 
Radar	|++	|++	|++	|-	|-	|++	|+
Lidar	|+	|++	|+	|+	|-	|+	|-


