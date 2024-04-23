/**
 * @description 基于computeTeethAutoBiteDistanceZ的方法进行碰撞检测。
 * 返回重叠面片的下标，将<0.1mm，0.1mm<0.3mm，>0.3mm的面片下标分别返回。
 * 计算下颌牙点比对应上颌牙面片高出的z轴距离。
 * 对于找出的下颌牙点，需要找到其所在的面片。
 * 注意pointDistanceWithPlaneAlongAxis沿z轴方向计算距离, 或许需要一定转换
 * @param toothPointsDatas 牙齿点, upper lower, 其中以牙齿名称作为key
 * @param toothFacesDatas 牙齿面片组成, upper lower, 其中以牙齿名称作为key
 * @param transMatrix 转换牙齿点到标准坐标系
 */
function detectCollisionsZ(
    toothPointsDatas,
    toothFacesDatas,
    transMatrix
){
    // console.time('转换耗时')
    // 构造下颌牙点集
    let numOfLowerToothPoints = 0;
    for (let toothName in toothPointsDatas) {
        if (typedBracketNameList.lower.includes(toothName)) {
            numOfLowerToothPoints += toothPointsDatas[toothName].length / 3;
        }
    }
    let pointsOfLowerTeeth = new Array(numOfLowerToothPoints),
    toothPointValues,
    pOffset = 0,
    arrLength;
    for (let toothName in toothPointsDatas) {
        if (typedBracketNameList.lower.includes(toothName)) {
            // ------------------------------------------------------------------------
            // 复制参数 (构造深拷贝)
            // ------------------------------------------------------------------------
            toothPointValues = toothPointsDatas[toothName];
            // 对牙齿点集应用变换
            vtkMatrixBuilder
                .buildFromDegree()
                .setMatrix(transMatrix[toothName])
                .apply(toothPointValues);
            arrLength = toothPointValues.length;
            for (
                let idxStart = 0, idxEnd = 3;
                idxStart < arrLength;
                idxStart += 3, idxEnd += 3
            ) {
                pointsOfLowerTeeth[pOffset++] = toothPointValues.subarray(
                    idxStart,
                    idxEnd
                );
            }
        }
    }
    // console.timeEnd('转换耗时') // 7

        // 计算下颌牙边界框
        let xminL = Infinity,
        yminL = Infinity,
        xmaxL = -Infinity,
        ymaxL = -Infinity;
    pointsOfLowerTeeth.forEach(([x, y, z]) => {
        if (xminL > x) {
            xminL = x;
        }
        if (xmaxL < x) {
            xmaxL = x;
        }
        if (yminL > y) {
            yminL = y;
        }
        if (ymaxL < y) {
            ymaxL = y;
        }
    });

    // console.time('构造面片耗时')
    // 构造上颌牙面片, 并计算面片的边界框
    // 计算上颌牙全面片数, 并预分配内存给facesOfUpperTeeth
    let numOfFaces = 0;
    for (let toothName in toothFacesDatas) {
        if (typedBracketNameList.upper.includes(toothName)) {
            numOfFaces += toothFacesDatas[toothName].length / 4;
        }
    }
    let facesOfUpperTeeth = new Array(numOfFaces),
        fbArrayOffset = 0,
        faceValues,
        point1Offset,
        point2Offset,
        point3Offset,
        point1,
        point2,
        point3;
    for (let toothName in toothFacesDatas) {
        if (typedBracketNameList.upper.includes(toothName)) {
            // 对牙齿点集应用变换
            toothPointValues = toothPointsDatas[toothName];
            vtkMatrixBuilder
                .buildFromDegree()
                .setMatrix(transMatrix[toothName])
                .apply(toothPointValues);
            // 读取面片构造
            faceValues = toothFacesDatas[toothName];
            numOfFaces = faceValues.length / 4;
            for (let fbOffset = 0; fbOffset < numOfFaces; fbOffset++) {
                point1Offset = faceValues[fbOffset + 1] * 3;
                point2Offset = faceValues[fbOffset + 2] * 3;
                point3Offset = faceValues[fbOffset + 3] * 3;
                point1 = toothPointValues.subarray(
                    point1Offset,
                    point1Offset + 3
                );
                point2 = toothPointValues.subarray(
                    point2Offset,
                    point2Offset + 3
                );
                point3 = toothPointValues.subarray(
                    point3Offset,
                    point3Offset + 3
                );
                facesOfUpperTeeth[fbArrayOffset++] = {
                    point1,
                    point2,
                    point3,
                    xmin: Math.min(point1[0], point2[0], point3[0]),
                    xmax: Math.max(point1[0], point2[0], point3[0]),
                    ymin: Math.min(point1[1], point2[1], point3[1]),
                    ymax: Math.max(point1[1], point2[1], point3[1]),
                };
            }
        }
    }
    // console.timeEnd('构造面片耗时') // 70

        // 计算上颌牙边界框
        let xminU = Infinity,
        yminU = Infinity,
        xmaxU = -Infinity,
        ymaxU = -Infinity;
    facesOfUpperTeeth.forEach(({ xmin, xmax, ymin, ymax }) => {
        if (xminU > xmin) {
            xminU = xmin;
        }
        if (xmaxU < xmax) {
            xmaxU = xmax;
        }
        if (yminU > ymin) {
            yminU = ymin;
        }
        if (ymaxU < ymax) {
            ymaxU = ymax;
        }
    });
    // 计算重合边界框
    let xminUL = Math.max(xminL, xminU),
        yminUL = Math.max(yminL, yminU),
        xmaxUL = Math.min(xmaxL, xmaxU),
        ymaxUL = Math.min(ymaxL, ymaxU);
    // 筛选
    pointsOfLowerTeeth = pointsOfLowerTeeth.filter(
        (val) =>
            val[0] > xminUL &&
            val[0] < xmaxUL &&
            val[1] > yminUL &&
            val[1] < ymaxUL
    );
    facesOfUpperTeeth = facesOfUpperTeeth.filter(
        ({ xmin, xmax, ymin, ymax }) =>
            Math.max(xmin, xminUL) < Math.min(xmax, xmaxUL) &&
            Math.max(ymin, yminUL) < Math.min(ymax, ymaxUL)
    );
    if (pointsOfLowerTeeth.length === 0 || facesOfUpperTeeth.length === 0) {
        return null;
    }

}

function solveLeastSquares(numberOfSamples, xt, xOrder, yt, yOrder, mt) {
    var checkHomogeneous = arguments.length > 6 && arguments[6] !== undefined ? arguments[6] : true;
  
    // check dimensional consistency
    if (numberOfSamples < xOrder || numberOfSamples < yOrder) {
      vtkWarningMacro('Insufficient number of samples. Underdetermined.');
      return 0;
    }
  
    var homogenFlags = createArray(yOrder);
    var allHomogeneous = 1;
    var hmt;
    var homogRC = 0;
    var i;
    var j;
    var k;
    var someHomogeneous = 0; // Ok, first init some flags check and see if all the systems are homogeneous
  
    if (checkHomogeneous) {
      // If Y' is zero, it's a homogeneous system and can't be solved via
      // the pseudoinverse method. Detect this case, warn the user, and
      // invoke SolveHomogeneousLeastSquares instead. Note that it doesn't
      // really make much sense for yOrder to be greater than one in this case,
      // since that's just yOrder occurrences of a 0 vector on the RHS, but
      // we allow it anyway. N
      // Initialize homogeneous flags on a per-right-hand-side basis
      for (j = 0; j < yOrder; j++) {
        homogenFlags[j] = 1;
      }
  
      for (i = 0; i < numberOfSamples; i++) {
        for (j = 0; j < yOrder; j++) {
          if (Math.abs(yt[i * yOrder + j]) > VTK_SMALL_NUMBER) {
            allHomogeneous = 0;
            homogenFlags[j] = 0;
          }
        }
      } // If we've got one system, and it's homogeneous, do it and bail out quickly.
  
  
      if (allHomogeneous && yOrder === 1) {
        vtkWarningMacro('Detected homogeneous system (Y=0), calling SolveHomogeneousLeastSquares()');
        return solveHomogeneousLeastSquares(numberOfSamples, xt, xOrder, mt);
      } // Ok, we've got more than one system of equations.
      // Figure out if we need to calculate the homogeneous equation solution for
      // any of them.
  
  
      if (allHomogeneous) {
        someHomogeneous = 1;
      } else {
        for (j = 0; j < yOrder; j++) {
          if (homogenFlags[j]) {
            someHomogeneous = 1;
          }
        }
      }
    } // If necessary, solve the homogeneous problem
  
  
    if (someHomogeneous) {
      // hmt is the homogeneous equation version of mt, the general solution.
      // hmt should be xOrder x yOrder, but since we are solving only the homogeneous part, here it is xOrder x 1
      hmt = createArray(xOrder); // Ok, solve the homogeneous problem
  
      homogRC = solveHomogeneousLeastSquares(numberOfSamples, xt, xOrder, hmt);
    } // set up intermediate variables
  
  
    var XXt = createArray(xOrder * xOrder); // size x by x
  
    var XXtI = createArray(xOrder * xOrder); // size x by x
  
    var XYt = createArray(xOrder * yOrder); // size x by y
    // first find the pseudoinverse matrix
  
    for (k = 0; k < numberOfSamples; k++) {
      for (i = 0; i < xOrder; i++) {
        // first calculate the XXt matrix, only do the upper half (symmetrical)
        for (j = i; j < xOrder; j++) {
          XXt[i * xOrder + j] += xt[k * xOrder + i] * xt[k * xOrder + j];
        } // now calculate the XYt matrix
  
  
        for (j = 0; j < yOrder; j++) {
          XYt[i * yOrder + j] += xt[k * xOrder + i] * yt[k * yOrder + j];
        }
      }
    } // now fill in the lower half of the XXt matrix
  
  
    for (i = 0; i < xOrder; i++) {
      for (j = 0; j < i; j++) {
        XXt[i * xOrder + j] = XXt[j * xOrder + i];
      }
    }
  
    var successFlag = invertMatrix(XXt, XXtI, xOrder); // next get the inverse of XXt
  
    if (successFlag) {
      for (i = 0; i < xOrder; i++) {
        for (j = 0; j < yOrder; j++) {
          mt[i * yOrder + j] = 0.0;
  
          for (k = 0; k < xOrder; k++) {
            mt[i * yOrder + j] += XXtI[i * xOrder + k] * XYt[k * yOrder + j];
          }
        }
      }
    } // Fix up any of the solutions that correspond to the homogeneous equation
    // problem.
  
  
    if (someHomogeneous) {
      for (j = 0; j < yOrder; j++) {
        if (homogenFlags[j]) {
          // Fix this one
          for (i = 0; i < xOrder; i++) {
            mt[i * yOrder + j] = hmt[i * yOrder];
          }
        }
      }
    }
  
    if (someHomogeneous) {
      return homogRC && successFlag;
    }
  
    return successFlag;
  }