import _defineProperty from '@babel/runtime/helpers/defineProperty';
import macro from '@kitware/vtk.js/macro.js';
import vtkAbstractPicker from './AbstractPicker.js';
import vtkBoundingBox from '@kitware/vtk.js/Common/DataModel/BoundingBox.js';
import { d as dot, l as normalize, n as norm, f as distance2BetweenPoints } from '@kitware/vtk.js/Common/Core/Math/index.js';
import { j as transpose, g as invert } from '../../vendor/gl-matrix/esm/mat4.js';
import { t as transformMat4 } from '../../vendor/gl-matrix/esm/vec4.js';

function ownKeys(object, enumerableOnly) { var keys = Object.keys(object); if (Object.getOwnPropertySymbols) { var symbols = Object.getOwnPropertySymbols(object); if (enumerableOnly) symbols = symbols.filter(function (sym) { return Object.getOwnPropertyDescriptor(object, sym).enumerable; }); keys.push.apply(keys, symbols); } return keys; }

function _objectSpread(target) { for (var i = 1; i < arguments.length; i++) { var source = arguments[i] != null ? arguments[i] : {}; if (i % 2) { ownKeys(Object(source), true).forEach(function (key) { _defineProperty(target, key, source[key]); }); } else if (Object.getOwnPropertyDescriptors) { Object.defineProperties(target, Object.getOwnPropertyDescriptors(source)); } else { ownKeys(Object(source)).forEach(function (key) { Object.defineProperty(target, key, Object.getOwnPropertyDescriptor(source, key)); }); } } return target; }
var vtkErrorMacro = macro.vtkErrorMacro;
var vtkWarningMacro = macro.vtkWarningMacro; // ----------------------------------------------------------------------------
// vtkPicker methods
// ----------------------------------------------------------------------------

function vtkPicker(publicAPI, model) {
  // Set our className
  model.classHierarchy.push('vtkPicker');

  var superClass = _objectSpread({}, publicAPI);

  function initialize() {
    superClass.initialize();
    model.actors = [];
    model.pickedPositions = [];
    model.mapperPosition[0] = 0.0;
    model.mapperPosition[1] = 0.0;
    model.mapperPosition[2] = 0.0;
    model.mapper = null;
    model.dataSet = null;
    model.globalTMin = Number.MAX_VALUE;
  } // Intersect data with specified ray.
  // Project the center point of the mapper onto the ray and determine its parametric value


  publicAPI.intersectWithLine = function (p1, p2, tol, mapper) {
    if (!mapper) {
      return Number.MAX_VALUE;
    }

    var center = mapper.getCenter();
    var ray = [];

    for (var i = 0; i < 3; i++) {
      ray[i] = p2[i] - p1[i];
    }

    var rayFactor = dot(ray, ray);

    if (rayFactor === 0.0) {
      return 2.0;
    } // Project the center point onto the ray and determine its parametric value


    var t = (ray[0] * (center[0] - p1[0]) + ray[1] * (center[1] - p1[1]) + ray[2] * (center[2] - p1[2])) / rayFactor;
    return t;
  }; // To be overridden in subclasses


  publicAPI.pick = function (selection, renderer) {
    if (selection.length !== 3) {
      vtkWarningMacro('vtkPicker::pick: selectionPt needs three components');
    }

    var selectionX = selection[0];
    var selectionY = selection[1];
    var selectionZ = selection[2];
    var cameraPos = [];
    var cameraFP = [];
    var displayCoords = [];
    var worldCoords = [];
    var ray = [];
    var cameraDOP = [];
    var clipRange = [];
    var tF;
    var tB;
    var p1World = [];
    var p2World = [];
    var viewport = [];
    var winSize = [];
    var x;
    var y;
    var windowLowerLeft = [];
    var windowUpperRight = [];
    var tol = 0.0;
    var props = [];
    var pickable = false;
    var p1Mapper = new Float64Array(4);
    var p2Mapper = new Float64Array(4);
    var bbox = vtkBoundingBox.newInstance();
    var t = [];
    var hitPosition = [];
    var view = renderer.getRenderWindow().getViews()[0];
    initialize();
    model.renderer = renderer;
    model.selectionPoint[0] = selectionX;
    model.selectionPoint[1] = selectionY;
    model.selectionPoint[2] = selectionZ;

    if (!renderer) {
      vtkErrorMacro('Picker::Pick Must specify renderer');
      return;
    } // Get camera focal point and position. Convert to display (screen)
    // coordinates. We need a depth value for z-buffer.


    var camera = renderer.getActiveCamera();
    cameraPos = camera.getPosition();
    cameraFP = camera.getFocalPoint();
    displayCoords = renderer.worldToNormalizedDisplay(cameraFP[0], cameraFP[1], cameraFP[2]);
    displayCoords = view.normalizedDisplayToDisplay(displayCoords[0], displayCoords[1], displayCoords[2]);
    selectionZ = displayCoords[2]; // Convert the selection point into world coordinates.

    var normalizedDisplay = view.displayToNormalizedDisplay(selectionX, selectionY, selectionZ);
    var dims = view.getViewportSize(renderer);
    var aspect = dims[0] / dims[1];
    worldCoords = renderer.normalizedDisplayToWorld(normalizedDisplay[0], normalizedDisplay[1], normalizedDisplay[2], aspect);

    for (var i = 0; i < 3; i++) {
      model.pickPosition[i] = worldCoords[i];
    } //  Compute the ray endpoints. The ray is along the line running from
    //  the camera position to the selection point, starting where this line
    //  intersects the front clipping plane, and terminating where this
    //  line intersects the back clipping plane.


    for (var _i = 0; _i < 3; _i++) {
      ray[_i] = model.pickPosition[_i] - cameraPos[_i];
    }

    for (var _i2 = 0; _i2 < 3; _i2++) {
      cameraDOP[_i2] = cameraFP[_i2] - cameraPos[_i2];
    }

    normalize(cameraDOP);
    var rayLength = dot(cameraDOP, ray);

    if (rayLength === 0.0) {
      vtkWarningMacro('Picker::Pick Cannot process points');
      return;
    }

    clipRange = camera.getClippingRange();

    if (camera.getParallelProjection()) {
      tF = clipRange[0] - rayLength;
      tB = clipRange[1] - rayLength;

      for (var _i3 = 0; _i3 < 3; _i3++) {
        p1World[_i3] = model.pickPosition[_i3] + tF * cameraDOP[_i3];
        p2World[_i3] = model.pickPosition[_i3] + tB * cameraDOP[_i3];
      }
    } else {
      tF = clipRange[0] / rayLength;
      tB = clipRange[1] / rayLength;

      for (var _i4 = 0; _i4 < 3; _i4++) {
        p1World[_i4] = cameraPos[_i4] + tF * ray[_i4];
        p2World[_i4] = cameraPos[_i4] + tB * ray[_i4];
      }
    }

    p1World[3] = 1.0;
    p2World[3] = 1.0; // Compute the tolerance in world coordinates.  Do this by
    // determining the world coordinates of the diagonal points of the
    // window, computing the width of the window in world coordinates, and
    // multiplying by the tolerance.

    viewport = renderer.getViewport();

    if (renderer.getRenderWindow()) {
      winSize = renderer.getRenderWindow().getViews()[0].getSize();
    }

    x = winSize[0] * viewport[0];
    y = winSize[1] * viewport[1];
    var normalizedLeftDisplay = view.displayToNormalizedDisplay(x, y, selectionZ);
    windowLowerLeft = renderer.normalizedDisplayToWorld(normalizedLeftDisplay[0], normalizedLeftDisplay[1], normalizedLeftDisplay[2], aspect);
    x = winSize[0] * viewport[2];
    y = winSize[1] * viewport[3];
    var normalizedRightDisplay = view.displayToNormalizedDisplay(x, y, selectionZ);
    windowUpperRight = renderer.normalizedDisplayToWorld(normalizedRightDisplay[0], normalizedRightDisplay[1], normalizedRightDisplay[2], aspect);

    for (var _i5 = 0; _i5 < 3; _i5++) {
      tol += (windowUpperRight[_i5] - windowLowerLeft[_i5]) * (windowUpperRight[_i5] - windowLowerLeft[_i5]);
    }

    tol = Math.sqrt(tol) * model.tolerance;

    if (model.pickFromList) {
      props = model.pickList;
    } else {
      props = renderer.getActors();
    }

    var scale = [];
    props.forEach(function (prop) {
      var mapper = prop.getMapper();
      pickable = prop.getPickable() && prop.getVisibility();

      if (prop.getProperty().getOpacity() <= 0.0) {
        pickable = false;
      }

      if (pickable) {
        model.transformMatrix = prop.getMatrix().slice(0); // Webgl need a transpose matrix but we need the untransposed one to project world points
        // into the right referential

        transpose(model.transformMatrix, model.transformMatrix);
        invert(model.transformMatrix, model.transformMatrix); // Extract scale

        var col1 = [model.transformMatrix[0], model.transformMatrix[1], model.transformMatrix[2]];
        var col2 = [model.transformMatrix[4], model.transformMatrix[5], model.transformMatrix[6]];
        var col3 = [model.transformMatrix[8], model.transformMatrix[9], model.transformMatrix[10]];
        scale[0] = norm(col1);
        scale[1] = norm(col2);
        scale[2] = norm(col3);
        transformMat4(p1Mapper, p1World, model.transformMatrix);
        transformMat4(p2Mapper, p2World, model.transformMatrix);
        p1Mapper[0] /= p1Mapper[3];
        p1Mapper[1] /= p1Mapper[3];
        p1Mapper[2] /= p1Mapper[3];
        p2Mapper[0] /= p2Mapper[3];
        p2Mapper[1] /= p2Mapper[3];
        p2Mapper[2] /= p2Mapper[3];

        for (var _i6 = 0; _i6 < 3; _i6++) {
          ray[_i6] = p2Mapper[_i6] - p1Mapper[_i6];
        }

        if (mapper) {
          bbox.setBounds(mapper.getBounds());
          bbox.inflate(tol);
        } else {
          bbox.reset();
        }

        if (bbox.intersectBox(p1Mapper, ray, hitPosition, t)) {
          t[0] = publicAPI.intersectWithLine(p1Mapper, p2Mapper, tol * 0.333 * (scale[0] + scale[1] + scale[2]), mapper);

          if (t[0] < Number.MAX_VALUE) {
            var p = [];
            p[0] = (1.0 - t[0]) * p1World[0] + t[0] * p2World[0];
            p[1] = (1.0 - t[0]) * p1World[1] + t[0] * p2World[1];
            p[2] = (1.0 - t[0]) * p1World[2] + t[0] * p2World[2]; // Check if the current actor is already in the list

            var actorID = -1;

            for (var _i7 = 0; _i7 < model.actors.length; _i7++) {
              if (model.actors[_i7] === prop) {
                actorID = _i7;
                break;
              }
            }

            if (actorID === -1) {
              model.actors.push(prop);
              model.pickedPositions.push(p);
            } else {
              var oldPoint = model.pickedPositions[actorID];
              var distOld = distance2BetweenPoints(p1World, oldPoint);
              var distCurrent = distance2BetweenPoints(p1World, p);

              if (distCurrent < distOld) {
                model.pickedPositions[actorID] = p;
              }
            }
          }
        }
      }

      publicAPI.invokePickChange(model.pickedPositions);
      return 1;
    });
  };
} // ----------------------------------------------------------------------------
// Object factory
// ----------------------------------------------------------------------------


var DEFAULT_VALUES = {
  tolerance: 0.025,
  mapperPosition: [0.0, 0.0, 0.0],
  mapper: null,
  dataSet: null,
  actors: [],
  pickedPositions: [],
  transformMatrix: null,
  globalTMin: Number.MAX_VALUE
}; // ----------------------------------------------------------------------------

function extend(publicAPI, model) {
  var initialValues = arguments.length > 2 && arguments[2] !== undefined ? arguments[2] : {};
  Object.assign(model, DEFAULT_VALUES, initialValues); // Inheritance

  vtkAbstractPicker.extend(publicAPI, model, initialValues);
  macro.setGet(publicAPI, model, ['tolerance']);
  macro.setGetArray(publicAPI, model, ['mapperPosition'], 3);
  macro.get(publicAPI, model, ['mapper', 'dataSet', 'actors', 'pickedPositions']);
  macro.event(publicAPI, model, 'pickChange');
  vtkPicker(publicAPI, model);
} // ----------------------------------------------------------------------------

var newInstance = macro.newInstance(extend, 'vtkPicker'); // ----------------------------------------------------------------------------

var vtkPicker$1 = {
  newInstance: newInstance,
  extend: extend
};

export default vtkPicker$1;
export { extend, newInstance };
