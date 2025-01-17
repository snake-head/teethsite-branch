import _toConsumableArray from '@babel/runtime/helpers/toConsumableArray';
import macro from '@kitware/vtk.js/macros.js';
import { vec3 } from 'gl-matrix';
import * as vtkMath from "@kitware/vtk.js/Common/Core/Math";
import { throttle } from "@kitware/vtk.js/macro";

function widgetBehavior(publicAPI, model) {
  var state = model.widgetState;
  var moveHandle = state.getMoveHandle();
  var centerHandle = state.getCenterHandle();
  var borderHandle = state.getBorderHandle();
  var shapeHandle = state.getSphereHandle(); // Set while moving the center or border handle.

  model._isDragging = false; // The last world coordinate of the mouse cursor during dragging.

  model.previousPosition = null;
  model.classHierarchy.push('vtkSphereWidgetProp');
  moveHandle.setVisible(true);
  centerHandle.setVisible(false);
  borderHandle.setVisible(false);
  shapeHandle.setVisible(true);

  function isValidHandle(handle) {
    return handle === centerHandle || handle === borderHandle || handle === moveHandle;
  }

  function isPlaced() {
    return !!centerHandle.getOrigin() && !!borderHandle.getOrigin();
  } // Update the sphereHandle parameters from {center,border}Handle.


  function updateSphere() {
    var center = centerHandle.getOrigin();
    if (!center) return;
    centerHandle.setVisible(true);
    var border = borderHandle.getOrigin();

    if (border) {
      borderHandle.setVisible(true);
    } else {
      border = moveHandle.getOrigin();
      if (!border) return;
    }

    if (isPlaced()) {
      moveHandle.setVisible(true);
    }

    var radius = vec3.distance(center, border);
    shapeHandle.setVisible(true);
    shapeHandle.setOrigin(center);
    shapeHandle.setScale1(radius * 2);

    model._interactor.render();
  }

  function currentWorldCoords(e) {
    var _model$activeState$ge, _model$activeState, _model$activeState$ge2;

    var manipulator = (_model$activeState$ge = (_model$activeState = model.activeState) === null || _model$activeState === void 0 ? void 0 : (_model$activeState$ge2 = _model$activeState.getManipulator) === null || _model$activeState$ge2 === void 0 ? void 0 : _model$activeState$ge2.call(_model$activeState)) !== null && _model$activeState$ge !== void 0 ? _model$activeState$ge : model.manipulator;
    return manipulator.handleEvent(e, model._apiSpecificRenderWindow);
  } // Update the sphere's center and radius.  Example:
  // handle.setCenterAndRadius([1,2,3], 10);


  publicAPI.setCenterAndRadius = function (newCenter, newRadius) {
    var oldCenter = centerHandle.getOrigin();
    var oldBorder = borderHandle.getOrigin();
    var newBorder = [newCenter[0] + newRadius, newCenter[1], newCenter[2]];

    if (oldBorder) {
      // Move the boundary handle to reflect the new radius, while preserving
      // its direction relative to the center.
      var direction = vec3.sub(vec3.create(), oldBorder, oldCenter);
      var oldRadius = vec3.length(direction);

      if (oldRadius > 1e-10) {
        newBorder = vec3.add(vec3.create(), newCenter, vec3.scale(vec3.create(), direction, newRadius / oldRadius));
      }
    }

    centerHandle.setOrigin(newCenter);
    borderHandle.setOrigin(newBorder);
    updateSphere();

    model._widgetManager.enablePicking();
  };

  // 2023.2.14更新：添加按钮来控制牙尖牙底小球的移动
  publicAPI.buttonMoveCenter = function (direction, normal, step=0.2, ifStraightenSimulation=false){
    // 基于法向量平面进行移动
    //2023.3.7更新：排牙状态下需要反向移动
    step = ifStraightenSimulation?-step:step;
    var oldCenter = centerHandle.getOrigin();
    var newCenter;
    if (direction=='UP'){
      newCenter = [oldCenter[0],oldCenter[1],oldCenter[2]+step];
    }else if(direction=='DOWN'){
      newCenter = [oldCenter[0],oldCenter[1],oldCenter[2]-step];
    }else if(direction=='RIGHT'){
      newCenter = [oldCenter[0]-normal[1]*step,oldCenter[1]+normal[0]*step,oldCenter[2]];
    }else if(direction=='LEFT'){
      newCenter = [oldCenter[0]+normal[1]*step,oldCenter[1]-normal[0]*step,oldCenter[2]];
    }
    newCenter = calNearestPoint(newCenter)
    centerHandle.setOrigin(newCenter);
    updateSphere();
    publicAPI.updatePosFunc(
      [...newCenter],
      model.renderer,
      model.renderWindow
    );
  }

  publicAPI.handleLeftButtonPress = function (e) {
    if (!isValidHandle(model.activeState)) {
      model.activeState = null;
      return macro.VOID;
    }
    console.time('鼠标按下')
    var worldCoords = currentWorldCoords(e);

    if (model.activeState === moveHandle) {
      // Initial sphere placement.
      if (!centerHandle.getOrigin()) {
        centerHandle.setOrigin(worldCoords);
      } else if (!borderHandle.getOrigin()) {
        borderHandle.setOrigin(worldCoords);
      }

      updateSphere();
    }

    model._isDragging = true;

    model._apiSpecificRenderWindow.setCursor('grabbing');

    model.previousPosition = _toConsumableArray(currentWorldCoords(e));
    publicAPI.invokeStartInteractionEvent();
    console.timeEnd('鼠标按下') // 2
    return macro.EVENT_ABORT;
  };

  publicAPI.handleLeftButtonRelease = function (e) {
    if (!model._isDragging) {
      model.activeState = null;
      return macro.VOID;
    }
    console.time('鼠标释放')
    if (isPlaced()) {
      model.previousPosition = null;

      model._widgetManager.enablePicking();

      model._apiSpecificRenderWindow.setCursor('pointer');

      model._isDragging = false;
      model.activeState = null;
      state.deactivate();
    }

    publicAPI.invokeEndInteractionEvent();
    console.timeEnd('鼠标释放') // 2
    return macro.EVENT_ABORT;
  };

  // 计算一点离依赖点集的最近点
  function calNearestPoint(p) {
    if(!model.dependingPoints){
      return p;
    }
    
    const sizeDependingPoints = model.dependingPoints.length;

    let nearestPoints = [
        model.dependingPoints[0],
        model.dependingPoints[1],
        model.dependingPoints[2],
    ];
    let minDist = vtkMath.distance2BetweenPoints(nearestPoints, p);
    for (let idx = 3; idx < sizeDependingPoints; idx += 3) {
        const pi = [
            model.dependingPoints[idx],
            model.dependingPoints[idx + 1],
            model.dependingPoints[idx + 2],
        ];
        const dist = vtkMath.distance2BetweenPoints(pi, p);
        if (dist < minDist) {
            minDist = dist;
            nearestPoints = pi;
        }
    }
    return nearestPoints;
  }
  publicAPI.handleMouseMove = (e)=> {
    if (!model._isDragging) {
      model.activeState = null;
      return macro.VOID;
    }
    console.time('鼠标移动')
    if (!model.activeState) throw Error('no activestate');
    var worldCoords = currentWorldCoords(e);
    worldCoords = calNearestPoint(worldCoords)
    model.activeState.setOrigin(worldCoords);
    
    // if (model.activeState === centerHandle) {
    //   // When the sphere is fully placed, and the user is moving the
    //   // center, we move the whole sphere.
    //   if (borderHandle.getOrigin()) {
    //     if (!model.previousPosition) {
    //       // !previousPosition here happens only immediately
    //       // after grabFocus, but grabFocus resets
    //       // borderHandle.origin.
    //       throw Error("no pos ".concat(model.activeState, " ").concat(model.previousPosition));
    //     }

    //     var translation = vec3.sub(vec3.create(), worldCoords, model.previousPosition);
    //     borderHandle.setOrigin(vec3.add(vec3.create(), borderHandle.getOrigin(), translation));
    //   }
    // }

    model.previousPosition = worldCoords;
    // borderHandle.setVisible(false);
    updateSphere();
    publicAPI.updatePosFunc(
      [...worldCoords],
      model.renderer,
      model.renderWindow
    );
    console.timeEnd('鼠标移动') // 60
    return macro.VOID;
  };

  publicAPI.grabFocus = function () {
    moveHandle.setVisible(true);
    centerHandle.setVisible(false);
    borderHandle.setVisible(false);
    centerHandle.setOrigin(null);
    borderHandle.setOrigin(null);
    model._isDragging = true;
    model.activeState = moveHandle;

    model._interactor.render();
  };

  publicAPI.loseFocus = function () {
    model._isDragging = false;
    model.activeState = null;
  };
}

export { widgetBehavior as default };
