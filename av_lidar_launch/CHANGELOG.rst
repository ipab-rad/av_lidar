^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package av_lidar_launch
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Update nebula version to improve OT128 support (`#7 <https://github.com/ipab-rad/av_lidar/issues/7>`_)
* Contributors: Alejandro Bordallo

1.0.0 (2025-08-11)
------------------
* Setup Hesai lidars (`#1 <https://github.com/ipab-rad/av_lidar/issues/1>`_)
  - Deprecate use of [nebula fork](https://github.com/ipab-rad/nebula) and
  instead wget release
  - Setup single "all_lidars" launch file to run all lidars together
  - Import and fix configuration from nebula_ros fork for both OT128 and
  QT128 for newest nebula release
  - Update Dockerfile + dev/runtime scripts to compile and copy
  compilation artefacts via multi-layer dockerfile
  ---------
  Co-authored-by: Hector Cruz <hector_cruz95@live.com>
* Contributors: Alejandro Bordallo
