// state本身是proxy, 其中的对象取出来也是proxy, 但取出其中的基本数据类型就是直接的值而不是proxy
// 在外部需要监视某个基本数据类型, 则需要套一层computed;
// 如果本来就是对象, 则不用再套一层computed去监视, 这样反而监视不到变化!
export default {
    namespaced: true,
    actions: {
        // 有异步操作咋action中进行
        updateSimMode(context, value) {
            context.commit("UpdateSimMode", value);
        },
        updateCurrentSelectBracketName(context, value) {
            context.commit("UpdateCurrentSelectBracketName", value);
        },
        updateCurrentMode(context, value) {
            context.commit("UpdateCurrentMode", value);
        },
        updateCurrentShowPanel(context, value) {
            context.commit("UpdateCurrentShowPanel", value);
        },
        updateTeethPositionAdjustType(context, value) {
            context.commit("UpdateTeethPositionAdjustType", value);
        },
        updateTeethPositionAdjustStep(context, value) {
            context.commit("UpdateTeethPositionAdjustStep", value);
        },
        updateTeethPositionAdjustAngle(context, value) {
            context.commit("UpdateTeethPositionAdjustAngle", value);
        },
        updateTeethPositionAdjustMoveType(context, value) {
            context.commit("UpdateTeethPositionAdjustMoveType", value);
        },
        updateSelectKeyBoardEvent(context, value) {
            context.commit("UpdateSelectKeyBoardEvent", value);
        },
        updateEnterAtInitTime(context, value) {
            context.commit("UpdateEnterAtInitTime", value);
        },
        updateTeethStandardAxis(context, value) {
            context.commit("UpdateTeethStandardAxis", value);
        },
        updateTeethAxisFinetuneRecord(context, value) {
            context.commit("UpdateTeethAxisFinetuneRecord", value);
        },
        updateDentalArchSettings(context, value) {
            context.commit("UpdateDentalArchSettings", value);
        },
        updateArrangeMatrix(context, value) {
            context.commit("UpdateArrangeMatrix", value);
        },
        updateStandardAxisActor(context, value) {
            context.commit("UpdateStandardAxisActor", value);
        },
        updateDentalArchAdjustRecord(context, value) {
            context.commit("UpdateDentalArchAdjustRecord", value);
        },
        saveDentalArchAdjustRecord(context) {
            context.commit("SaveDentalArchAdjustRecord");
        },
        saveAdjustWidgetCenters(context, value) {
            context.commit("SaveAdjustWidgetCenters", value);
        },
        updateDentalArchLockState(context, value) {
            context.commit("UpdateDentalArchLockState", value);
        },
    },
    mutations: {
        UpdateSimMode(state, value) {
            state.simMode = value;
        },
        UpdateCurrentSelectBracketName(state, value) {
            state.currentSelectBracketName = value;
        },
        UpdateCurrentMode(state, value) {
            for (let mode in value) {
                state.currentMode[mode] = value[mode];
            }
        },
        UpdateCurrentShowPanel(state, value) {
            state.currentShowPanel = value;
        },
        UpdateTeethPositionAdjustType(state, value) {
            state.teethPositionAdjust.teethType = value;
        },
        UpdateTeethPositionAdjustStep(state, value) {
            state.teethPositionAdjust.step = value;
        },
        UpdateTeethPositionAdjustAngle(state, value) {
            state.teethPositionAdjust.angle = value;
        },
        UpdateTeethPositionAdjustMoveType(state, value) {
            state.teethPositionAdjustMoveType = value;
        },
        UpdateSelectKeyBoardEvent(state, value) {
            state.selectKeyBoardEvent = value;
        },
        UpdateEnterAtInitTime(state, value) {
            state.teethArrange.enterAtInitTime = value;
        },
        UpdateTeethStandardAxis(state, value) {
            Object.assign(state.teethArrange.teethStandardAxis, value);
        },
        UpdateTeethAxisFinetuneRecord(state, value) {
            Object.assign(state.teethArrange.teethAxisFinetuneRecord, value);
        },
        UpdateDentalArchSettings(state, value) {
            for (let teethType in value) {
                Object.assign(
                    state.teethArrange.dentalArchSettings[teethType],
                    value[teethType]
                );
            }
        },
        UpdateArrangeMatrix(state, value) {
            Object.assign(state.teethArrange.arrangeMatrix, value);
        },
        UpdateStandardAxisActor(state, value) {
            Object.assign(state.teethActors.standardAxisActor, value);
        },
        UpdateDentalArchAdjustRecord(state, value) {
            let valueProps = Object.keys(value);
            for (let recordProps of [
                "reCalculateArch",
                "teethType",
                "overwriteByDentalArchAdjustRecord",
                "regenerate",
                "isReinitActivate",
            ]) {
                if (valueProps.includes(recordProps)) {
                    state.teethArrange.dentalArchAdjustRecord[recordProps] =
                        value[recordProps];
                }
            }
            for (let teethType of ["upper", "lower"]) {
                if (valueProps.includes(teethType)) {
                    let valueTypeProps = Object.keys(value[teethType]);
                    for (let recordProps of ["centers"]) {
                        if (valueTypeProps.includes(recordProps)) {
                            Object.assign(
                                state.teethArrange.dentalArchAdjustRecord[
                                    teethType
                                ][recordProps],
                                value[teethType][recordProps]
                            );
                        }
                    }
                    for (let recordProps of [
                        "reset",
                        "reArrange",
                        "coEfficients",
                        "arrangeMatrix",
                        "reArrangeToInitState",
                    ]) {
                        if (valueTypeProps.includes(recordProps)) {
                            state.teethArrange.dentalArchAdjustRecord[
                                teethType
                            ][recordProps] = value[teethType][recordProps];
                            if (recordProps === "coEfficients") {
                                state.teethArrange.dentalArchAdjustRecord[
                                    teethType
                                ].isArrangeUpdated = false;
                            }
                            if (recordProps === "arrangeMatrix") {
                                state.teethArrange.dentalArchAdjustRecord[
                                    teethType
                                ].isArrangeUpdated = true;
                            }
                        }
                    }
                    if (value[teethType].reset === true) {
                        Object.assign(
                            state.teethArrange.dentalArchAdjustRecord[
                                teethType
                            ],
                            {
                                isArrangeUpdated: true,
                                coEfficients: null,
                                centers: {}, // centers设置成最开始的状态, 由外部把重置点搬过来
                                arrangeMatrix: {},
                            }
                        );
                    }
                    if (value[teethType].reArrangeToInitState === true) {
                        Object.assign(
                            state.teethArrange.dentalArchAdjustRecord[
                                teethType
                            ],
                            {
                                isArrangeUpdated: true, // 是否在最新调整的牙弓线上排过牙, 为false时不能保存
                                coEfficients: null, // 调整后拟合的牙弓线系数
                                centers: {}, // centers设置成最开始的状态,
                                arrangeMatrix: {}, // 在调整后的牙弓线上排牙的结果
                                resetCenters: {},
                            }
                        );
                    }
                }
            }
        },
        SaveDentalArchAdjustRecord(state) {
            // coEfficients覆盖出去
            for (let teethType of ["upper", "lower"]) {
                let {
                    coEfficients,
                } = state.teethArrange.dentalArchAdjustRecord[teethType];
                if (coEfficients !== null) {
                    state.teethArrange.dentalArchSettings[
                        teethType
                    ].coEfficients = coEfficients;
                }
            }
            // arrangeMatrix覆盖出去
            Object.assign(state.teethArrange.arrangeMatrix, {
                ...state.teethArrange.dentalArchAdjustRecord.upper
                    .arrangeMatrix,
                ...state.teethArrange.dentalArchAdjustRecord.lower
                    .arrangeMatrix,
            });
            // // 小球当前的位置设置为重置点, 以后重新生成小球时就直接从这个记录里读取
            // // 但是还要存一个矩阵, 所以新开一个函数, 外部调用
            // for (let teethType of ["upper", "lower"]) {
            //     Object.assign(
            //         state.teethArrange.dentalArchAdjustRecord[teethType].resetCenters,
            //         state.teethArrange.dentalArchAdjustRecord[teethType].centers
            //     );
            // }
            // 重置其中的参数为null
            for (let teethType of ["upper", "lower"]) {
                Object.assign(
                    state.teethArrange.dentalArchAdjustRecord[teethType],
                    {
                        isReinitActivate: true, // 此时可以点击[初始化]
                        isArrangeUpdated: true,
                        coEfficients: null,
                        centers: {},
                        arrangeMatrix: {},
                    }
                );
            }

            // 然后设置overwriteByDentalArchAdjustRecord为true, 引起外部监听事件
            state.teethArrange.dentalArchAdjustRecord.overwriteByDentalArchAdjustRecord = true;
            // 主要触发userMatrix的变化
        },
        SaveAdjustWidgetCenters(state, value) {
            // 保存时, 外部调用
            // 外部计算此时各个牙弓线调整小球的中心点
            // 并将其存入, 供下次生成时直接读取
            // 小球当前位置centers写入resetCenters, 此后生成小球, 重置点将不再依据托槽位置生成, 而是直接读取resetCenters
            Object.assign(
                state.teethArrange.dentalArchAdjustRecord.upper.resetCenters,
                value.upper
            );
            Object.assign(
                state.teethArrange.dentalArchAdjustRecord.lower.resetCenters,
                value.lower
            );
        },
        UpdateDentalArchLockState(state, value) {
            state.teethArrange.lockDentalArch = value;
        },
    },
    state: {
        simMode: "simBracketFix", //"simToothFix",
        currentSelectBracketName: "",

        currentMode: {
            fineTune: false,
            straightenSimulation: false, // 模拟矫正
        },
        // 牙齿位置调整设置
        currentShowPanel: -1, // 当前显示菜单, -1为工具菜单
        teethPositionAdjust: {
            teethType: "upper",
            step: 0.2,
            angle: 3.0,
        },
        // 外部监听,调用对应事件， 牙齿位置更新, 更新完毕重置为""
        // 如果不为""则不更新为其它值
        teethPositionAdjustMoveType: "",
        // 绑定的键盘事件, 目前在viewerMain中和Viewer中监听
        selectKeyBoardEvent: "",
        teethArrange: {
            enterAtInitTime: true, // 首次加载, 满足条件, 则直接进入排牙, 否则直接置为false
            teethStandardAxis: {
                upper: {
                    center: null,
                    xNormal: null,
                    yNormal: null,
                    zNormal: null,
                },
                lower: {
                    center: null,
                    xNormal: null,
                    yNormal: null,
                    zNormal: null,
                },
            }, // 初始化时文件里有就直接读, 否则通过后续子线程计算得到, 会保存至文件中
            teethAxisFinetuneRecord: {
                upper: {
                    center: null,
                    xNormal: null,
                    yNormal: null,
                    zNormal: null,
                },
                lower: {
                    center: null,
                    xNormal: null,
                    yNormal: null,
                    zNormal: null,
                },
            }, // 初始化时文件里有就直接读, 否则后续根据标准坐标系初始化, 用于后续咬合调整
            // 注意这里保存的值只用于重置咬合关系时读取, 或者用于初始化, 后续咬合关系调整不在此处更新
            // (递交时倒是可更新)
            dentalArchSettings: {
                upper: {
                    coEfficients: null,
                    W: undefined,
                    axisCoord: undefined,
                    zLevelOfArch: undefined,
                },
                lower: {
                    coEfficients: null,
                    W: undefined,
                    axisCoord: undefined,
                    zLevelOfArch: undefined,
                },
            },
            lockDentalArch: false, // 是否锁定当前牙弓线参数(影响排牙, 锁定后不会重新生成牙弓线, 而是直接在现有牙弓线基础上排牙)
            arrangeMatrix: {}, // 排牙矩阵
            dentalArchAdjustRecord: {
                teethType: "upper",
                reCalculateArch: false,
                overwriteByDentalArchAdjustRecord: false,
                regenerate: false,
                upper: {
                    reArrangeToInitState: false, // 点击[初始化]后设置为true, 用于触发viewerMain中的更新
                    isReinitActivate: false, // 是否能[初始化], 只有点了一次[保存]覆盖外部牙弓线以后才会设置为true
                    isArrangeUpdated: true, // 是否在最新调整的牙弓线上排过牙, 为false时不能保存
                    // 初始化为true, 重置后为true, 在调整牙弓线coEfficients后设置为false, 在arrangeMatrix更新后设置为true
                    reset: false, // 重置
                    reArrange: false, // 根据当前调整牙弓线排牙
                    coEfficients: null, // 调整后拟合的牙弓线系数
                    centers: {}, // 小球经用户拖动后的中心点
                    arrangeMatrix: {}, // 在调整后的牙弓线上排牙的结果
                    // 小球重置点, 每次生成小球时, resetCenters则直接读取此处的中心点, 否则根据托槽位置计算
                    // 该记录会保存至服务器文件, 下次初始化时会读取
                    resetCenters: {},
                },
                lower: {
                    reArrangeToInitState: false, // 点击[初始化]后设置为true, 用于触发viewerMain中的更新
                    isReinitActivate: false, // 是否能[初始化], 只有点了一次[保存]覆盖外部牙弓线以后才会设置为true
                    isArrangeUpdated: true, // 是否在最新调整的牙弓线上排过牙, 为false时不能保存
                    // 初始化为true, 重置后为true, 在调整牙弓线coEfficients后设置为false, 在arrangeMatrix更新后设置为true
                    reset: false, // 重置
                    reArrange: false, // 根据当前调整牙弓线排牙
                    coEfficients: null, // 调整后拟合的牙弓线系数
                    centers: {}, // 小球经用户拖动后的中心点
                    arrangeMatrix: {}, // 在调整后的牙弓线上排牙的结果
                    // 小球重置点, 每次生成小球时, resetCenters则直接读取此处的中心点, 否则根据托槽位置计算
                    // 该记录会保存至服务器文件, 下次初始化时会读取
                    resetCenters: {},
                },
            },
        },
        teethActors: {
            standardAxisActor: { actor: null, mapper: null }, // 根据标准坐标系计算, 仅需计算一次
        },
    },
    getters: {
        fineTuneMode(state) {
            if (!state.currentMode.straightenSimulation) {
                return "normal";
            }
            return state.simMode;
        },
        isArrangeDataComplete(state) {
            // 是否当前数据直接满足排牙条件, 配合actorLoadedFinish使用
            return Object.keys(state.teethArrange.arrangeMatrix).length > 0;
        },
        mergedDentalArchCoefficients(state) {
            // 返回当前的牙弓线参数, 调整后牙弓线参数 > 调整前牙弓线参数
            const {
                upper: { coEfficients: uCoefSetting },
                lower: { coEfficients: lCoefSetting },
            } = state.dentalArchSettings;
            const {
                upper: { coEfficients: uCoefAdjust },
                lower: { coEfficients: lCoefAdjust },
            } = state.teethArrange.dentalArchAdjustRecord;
            return {
                upper: uCoefAdjust ? uCoefAdjust : uCoefSetting,
                lower: lCoefAdjust ? lCoefAdjust : lCoefSetting,
            };
        },
        mergedArrangeMatrix(state) {
            // 返回当前的排牙矩阵, 调整后牙弓线的排牙矩阵 > 调整前排牙矩阵
            return Object.assign({}, state.teethArrange.arrangeMatrix, {
                ...state.teethArrange.dentalArchAdjustRecord.upper
                    .arrangeMatrix,
                ...state.teethArrange.dentalArchAdjustRecord.lower
                    .arrangeMatrix,
            });
        },
        isResetAdjustDentalArch(state) {
            return {
                upper: state.teethArrange.dentalArchAdjustRecord.upper.reset,
                lower: state.teethArrange.dentalArchAdjustRecord.lower.reset,
            };
        },
        isResetDentalArchToInitState(state) {
            return {
                upper:
                    state.teethArrange.dentalArchAdjustRecord.upper
                        .reArrangeToInitState,
                lower:
                    state.teethArrange.dentalArchAdjustRecord.lower
                        .reArrangeToInitState,
            };
        },
        isReArrangeTeethByAdjustedDentalArch(state) {
            return {
                upper:
                    state.teethArrange.dentalArchAdjustRecord.upper.reArrange,
                lower:
                    state.teethArrange.dentalArchAdjustRecord.lower.reArrange,
            };
        },
        isArrangedOnLatestAdjustDentalArch(state) {
            // 当前选择的颌牙是否在调整过的最新牙弓线上排过牙
            return state.teethArrange.dentalArchAdjustRecord[
                state.teethArrange.dentalArchAdjustRecord.teethType
            ].isArrangeUpdated;
        },
        canUserSaveAdjustRecord(state, _1, _2, allGetters) {
            // 1. 上下颌牙均在调整过的最新牙弓线上排过牙(every)
            // 2. 当前调整的coefficients至少有一个不为null(any)
            // 是的话则可以【保存】
            let isAdjusted = false;
            for (let teethType of allGetters[
                "userHandleState/arrangeTeethType"
            ]) {
                let {
                    isArrangeUpdated,
                    coEfficients,
                } = state.teethArrange.dentalArchAdjustRecord[teethType];
                if (coEfficients !== null) {
                    // 任一为true(调整过), 即可保存
                    isAdjusted = true;
                }
                if (!isArrangeUpdated) {
                    // 任一为false, 则不可保存
                    return false;
                }
            }
            return isAdjusted;
        },
    },
};
