import _defineProperty from '@babel/runtime/helpers/defineProperty';
import { f as distance2BetweenPoints } from '@kitware/vtk.js/Common/Core/Math/index.js';
import macro from '@kitware/vtk.js/macros.js';
import generateState from './LineWidget/state.js';
import vtkAbstractWidgetFactory from '../Core/AbstractWidgetFactory.js';
import vtkArrowHandleRepresentation from '@kitware/vtk.js/Widgets/Representations/ArrowHandleRepresentation.js';
import vtkPlanePointManipulator from '@kitware/vtk.js/Widgets/Manipulators/PlaneManipulator.js';
// import vtkSVGLandmarkRepresentation from '@kitware/vtk.js/Widgets/SVG/SVGLandmarkRepresentation.js';
import vtkPolyLineRepresentation from '@kitware/vtk.js/Widgets/Representations/PolyLineRepresentation.js';
import widgetBehavior from './LineWidget/behavior.js';
import { Behavior } from '../Representations/WidgetRepresentation/Constants.js';
import { ViewTypes } from '../Core/WidgetManager/Constants.js';
import { getPoint, updateTextPosition } from './LineWidget/helpers.js';

function ownKeys(object, enumerableOnly) { var keys = Object.keys(object); if (Object.getOwnPropertySymbols) { var symbols = Object.getOwnPropertySymbols(object); enumerableOnly && (symbols = symbols.filter(function (sym) { return Object.getOwnPropertyDescriptor(object, sym).enumerable; })), keys.push.apply(keys, symbols); } return keys; }

function _objectSpread(target) { for (var i = 1; i < arguments.length; i++) { var source = null != arguments[i] ? arguments[i] : {}; i % 2 ? ownKeys(Object(source), !0).forEach(function (key) { _defineProperty(target, key, source[key]); }) : Object.getOwnPropertyDescriptors ? Object.defineProperties(target, Object.getOwnPropertyDescriptors(source)) : ownKeys(Object(source)).forEach(function (key) { Object.defineProperty(target, key, Object.getOwnPropertyDescriptor(source, key)); }); } return target; }
// Factory
// ----------------------------------------------------------------------------

function vtkLineWidget(publicAPI, model) {
  model.classHierarchy.push('vtkLineWidget');

  var superClass = _objectSpread({}, publicAPI); // --- Widget Requirement ---------------------------------------------------


  model.methodsToLink = ['activeScaleFactor', 'activeColor', 'useActiveColor', 'glyphResolution', 'defaultScale', 'scaleInPixels'];

  publicAPI.getRepresentationsForViewType = function (viewType) {
    switch (viewType) {
      case ViewTypes.DEFAULT:
      case ViewTypes.GEOMETRY:
      case ViewTypes.SLICE:
      case ViewTypes.VOLUME:
      default:
        return [{
          builder: vtkArrowHandleRepresentation,
          labels: ['handle1'],
          initialValues: {
            /*
             * This table sets the visibility of the handles' actors
             * 1st actor is a displayActor, which hides a rendered object on the HTML layer.
             * operating on its value allows to hide a handle to the user while still being
             * able to detect its presence, so the user can move it. 2nd actor is a classic VTK
             * actor which renders the object on the VTK scene
             */
            visibilityFlagArray: [false, false],
            coincidentTopologyParameters: {
              Point: {
                factor: -1.0,
                offset: -1.0
              },
              Line: {
                factor: -1.0,
                offset: -1.0
              },
              Polygon: {
                factor: -3.0,
                offset: -3.0
              }
            }
          }
        }, {
          builder: vtkArrowHandleRepresentation,
          labels: ['handle2'],
          initialValues: {
            /*
             * This table sets the visibility of the handles' actors
             * 1st actor is a displayActor, which hides a rendered object on the HTML layer.
             * operating on its value allows to hide a handle to the user while still being
             * able to detect its presence, so the user can move it. 2nd actor is a classic VTK
             * actor which renders the object on the VTK scene
             */
            visibilityFlagArray: [false, false],
            coincidentTopologyParameters: {
              Point: {
                factor: -1.0,
                offset: -1.0
              },
              Line: {
                factor: -1.0,
                offset: -1.0
              },
              Polygon: {
                factor: -3.0,
                offset: -3.0
              }
            }
          }
        }, 
        {
          builder: vtkArrowHandleRepresentation,
          labels: ['moveHandle'],
          initialValues: {
            visibilityFlagArray: [false, false],
            coincidentTopologyParameters: {
              Point: {
                factor: -1.0,
                offset: -1.0
              },
              Line: {
                factor: -1.0,
                offset: -1.0
              },
              Polygon: {
                factor: -3.0,
                offset: -3.0
              }
            }
          }
        }, 
        // {
        //   builder: vtkSVGLandmarkRepresentation,
        //   initialValues: {
        //     text: '',
        //     textProps: {
        //       dx: 12,
        //       dy: -12
        //     }
        //   },
        //   labels: ['SVGtext']
        // }, 
        {
          builder: vtkPolyLineRepresentation,
          labels: ['handle1', 'handle2', 'moveHandle'],
          initialValues: {
            behavior: Behavior.HANDLE,
            pickable: true
          }
        }];
    }
  }; // --- Public methods -------------------------------------------------------


  publicAPI.getDistance = function () {
    var p1 = getPoint(0, model.widgetState);
    var p2 = getPoint(1, model.widgetState);
    return p1 && p2 ? Math.sqrt(distance2BetweenPoints(p1, p2)) : 0;
  };

  publicAPI.setManipulator = function (manipulator) {
    superClass.setManipulator(manipulator);
    model.widgetState.getMoveHandle().setManipulator(manipulator);
    model.widgetState.getHandle1().setManipulator(manipulator);
    model.widgetState.getHandle2().setManipulator(manipulator);
  }; // --------------------------------------------------------------------------
  // initialization
  // --------------------------------------------------------------------------

  /**
   * TBD: Why setting the move handle ?
   */


  model.widgetState.onBoundsChange(function (bounds) {
    var center = [(bounds[0] + bounds[1]) * 0.5, (bounds[2] + bounds[3]) * 0.5, (bounds[4] + bounds[5]) * 0.5];
    model.widgetState.getMoveHandle().setOrigin(center);
  });
  model.widgetState.getPositionOnLine().onModified(function () {
    updateTextPosition(model);
  }); // Default manipulator

  publicAPI.setManipulator(model.manipulator || vtkPlanePointManipulator.newInstance({
    useCameraNormal: true
  }));
} // ----------------------------------------------------------------------------


var defaultValues = function defaultValues(initialValues) {
  return _objectSpread({
    // manipulator: null,
    behavior: widgetBehavior,
    widgetState: generateState()
  }, initialValues);
}; // ----------------------------------------------------------------------------


function extend(publicAPI, model) {
  var initialValues = arguments.length > 2 && arguments[2] !== undefined ? arguments[2] : {};
  Object.assign(model, defaultValues(initialValues));
  vtkAbstractWidgetFactory.extend(publicAPI, model, initialValues);
  macro.setGet(publicAPI, model, ['manipulator']);
  vtkLineWidget(publicAPI, model);
} // ----------------------------------------------------------------------------

var newInstance = macro.newInstance(extend, 'vtkLineWidget'); // ----------------------------------------------------------------------------

var vtkLineWidget$1 = {
  newInstance: newInstance,
  extend: extend
};

export { vtkLineWidget$1 as default, extend, newInstance };
