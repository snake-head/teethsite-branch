import macro from '@kitware/vtk.js/macros.js';
import vtkInteractorObserver from '@kitware/vtk.js/Rendering/Core/InteractorObserver.js';
import vtkProp from '@kitware/vtk.js/Rendering/Core/Prop.js';
import { RenderingTypes } from './WidgetManager/Constants.js';
import { WIDGET_PRIORITY } from './AbstractWidget/Constants.js';

function vtkAbstractWidget(publicAPI, model) {
  model.classHierarchy.push('vtkAbstractWidget');
  model.actorToRepresentationMap = new WeakMap(); // --------------------------------------------------------------------------

  publicAPI.getBounds = model.widgetState.getBounds;

  publicAPI.getNestedProps = function () {
    return model.representations;
  }; // --------------------------------------------------------------------------


  publicAPI.activateHandle = function (_ref) {
    var selectedState = _ref.selectedState,
        representation = _ref.representation;
    // model.widgetState.activateOnly(selectedState);
    model.activeState = selectedState;

    if (selectedState && selectedState.updateManipulator) {
      selectedState.updateManipulator();
    }

    publicAPI.invokeActivateHandle({
      selectedState: selectedState,
      representation: representation
    });

    if (publicAPI.updateCursor) {
      publicAPI.updateCursor();
    }
  }; // --------------------------------------------------------------------------


  publicAPI.deactivateAllHandles = function () {
    // model.widgetState.deactivate();
  }; // --------------------------------------------------------------------------


  publicAPI.hasActor = function (actor) {
    return model.actorToRepresentationMap.has(actor);
  }; // --------------------------------------------------------------------------


  publicAPI.grabFocus = function () {
    model.hasFocus = true;
  };

  publicAPI.loseFocus = function () {
    model.hasFocus = false;
  };

  publicAPI.hasFocus = function () {
    return model.hasFocus;
  }; // --------------------------------------------------------------------------


  publicAPI.placeWidget = function (bounds) {
    return model.widgetState.placeWidget(bounds);
  };

  publicAPI.getPlaceFactor = function () {
    return model.widgetState.getPlaceFactor();
  };

  publicAPI.setPlaceFactor = function (factor) {
    return model.widgetState.setPlaceFactor(factor);
  }; // --------------------------------------------------------------------------


  publicAPI.getRepresentationFromActor = function (actor) {
    return model.actorToRepresentationMap.get(actor);
  }; // --------------------------------------------------------------------------


  publicAPI.updateRepresentationForRender = function () {
    var renderingType = arguments.length > 0 && arguments[0] !== undefined ? arguments[0] : RenderingTypes.FRONT_BUFFER;

    for (var i = 0; i < model.representations.length; i++) {
      var representation = model.representations[i];
      representation.updateActorVisibility(renderingType, model.contextVisibility, model.handleVisibility);
    }
  };

  publicAPI.getViewWidgets = function () {
    return model._factory.getViewIds().map(function (viewId) {
      return model._factory.getWidgetForView({
        viewId: viewId
      });
    });
  }; // --------------------------------------------------------------------------
  // Initialization calls
  // --------------------------------------------------------------------------


  publicAPI.setPriority(WIDGET_PRIORITY);
} // ----------------------------------------------------------------------------


var DEFAULT_VALUES = {
  contextVisibility: true,
  handleVisibility: true,
  hasFocus: false
};
/**
 * @param {*} publicAPI public methods to populate
 * @param {*} model internal values to populate
 * @param {object} initialValues Contains at least
 *   {viewType, _renderer, _camera, _openGLRenderWindow, _factory}
 */

function extend(publicAPI, model) {
  var initialValues = arguments.length > 2 && arguments[2] !== undefined ? arguments[2] : {};
  Object.assign(model, DEFAULT_VALUES, initialValues);
  vtkProp.extend(publicAPI, model, initialValues);
  vtkInteractorObserver.extend(publicAPI, model, initialValues);
  macro.setGet(publicAPI, model, ['contextVisibility', 'handleVisibility', '_widgetManager']);
  macro.get(publicAPI, model, ['representations', 'widgetState']);
  macro.moveToProtected(publicAPI, model, ['widgetManager']);
  macro.event(publicAPI, model, 'ActivateHandle');
  vtkAbstractWidget(publicAPI, model);
} // ----------------------------------------------------------------------------

var newInstance = macro.newInstance(extend, 'vtkAbstractWidget'); // ----------------------------------------------------------------------------

var vtkAbstractWidget$1 = {
  newInstance: newInstance,
  extend: extend
};

export { vtkAbstractWidget$1 as default, extend, newInstance };
