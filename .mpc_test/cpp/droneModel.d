droneModel.o: droneModel.cpp lib/Eigen/Dense lib/Eigen/Core \
 lib/Eigen/src/Core/util/DisableStupidWarnings.h \
 lib/Eigen/src/Core/util/Macros.h lib/Eigen/src/Core/util/MKL_support.h \
 lib/Eigen/src/Core/util/Constants.h lib/Eigen/src/Core/util/Meta.h \
 lib/Eigen/src/Core/util/ForwardDeclarations.h \
 lib/Eigen/src/Core/util/StaticAssert.h \
 lib/Eigen/src/Core/util/XprHelper.h lib/Eigen/src/Core/util/Memory.h \
 lib/Eigen/src/Core/util/IntegralConstant.h \
 lib/Eigen/src/Core/util/SymbolicIndex.h lib/Eigen/src/Core/NumTraits.h \
 lib/Eigen/src/Core/MathFunctions.h \
 lib/Eigen/src/Core/GenericPacketMath.h \
 lib/Eigen/src/Core/MathFunctionsImpl.h \
 lib/Eigen/src/Core/arch/SSE/PacketMath.h \
 lib/Eigen/src/Core/arch/SSE/MathFunctions.h \
 lib/Eigen/src/Core/arch/SSE/Complex.h \
 lib/Eigen/src/Core/arch/SSE/TypeCasting.h \
 lib/Eigen/src/Core/arch/CUDA/Half.h \
 lib/Eigen/src/Core/arch/CUDA/PacketMathHalf.h \
 lib/Eigen/src/Core/arch/CUDA/TypeCasting.h \
 lib/Eigen/src/Core/arch/Default/Settings.h \
 lib/Eigen/src/Core/functors/TernaryFunctors.h \
 lib/Eigen/src/Core/functors/BinaryFunctors.h \
 lib/Eigen/src/Core/functors/UnaryFunctors.h \
 lib/Eigen/src/Core/functors/NullaryFunctors.h \
 lib/Eigen/src/Core/functors/StlFunctors.h \
 lib/Eigen/src/Core/functors/AssignmentFunctors.h \
 lib/Eigen/src/Core/arch/CUDA/Complex.h \
 lib/Eigen/src/Core/util/IndexedViewHelper.h \
 lib/Eigen/src/Core/ArithmeticSequence.h lib/Eigen/src/Core/IO.h \
 lib/Eigen/src/Core/DenseCoeffsBase.h lib/Eigen/src/Core/DenseBase.h \
 lib/Eigen/src/Core/../plugins/CommonCwiseUnaryOps.h \
 lib/Eigen/src/Core/../plugins/BlockMethods.h \
 lib/Eigen/src/Core/../plugins/IndexedViewMethods.h \
 lib/Eigen/src/Core/../plugins/IndexedViewMethods.h \
 lib/Eigen/src/Core/MatrixBase.h \
 lib/Eigen/src/Core/../plugins/CommonCwiseBinaryOps.h \
 lib/Eigen/src/Core/../plugins/MatrixCwiseUnaryOps.h \
 lib/Eigen/src/Core/../plugins/MatrixCwiseBinaryOps.h \
 lib/Eigen/src/Core/EigenBase.h lib/Eigen/src/Core/Product.h \
 lib/Eigen/src/Core/CoreEvaluators.h lib/Eigen/src/Core/AssignEvaluator.h \
 lib/Eigen/src/Core/Assign.h lib/Eigen/src/Core/ArrayBase.h \
 lib/Eigen/src/Core/../plugins/ArrayCwiseUnaryOps.h \
 lib/Eigen/src/Core/../plugins/ArrayCwiseBinaryOps.h \
 lib/Eigen/src/Core/util/BlasUtil.h lib/Eigen/src/Core/DenseStorage.h \
 lib/Eigen/src/Core/NestByValue.h lib/Eigen/src/Core/ReturnByValue.h \
 lib/Eigen/src/Core/NoAlias.h lib/Eigen/src/Core/PlainObjectBase.h \
 lib/Eigen/src/Core/Matrix.h lib/Eigen/src/Core/Array.h \
 lib/Eigen/src/Core/CwiseTernaryOp.h lib/Eigen/src/Core/CwiseBinaryOp.h \
 lib/Eigen/src/Core/CwiseUnaryOp.h lib/Eigen/src/Core/CwiseNullaryOp.h \
 lib/Eigen/src/Core/CwiseUnaryView.h \
 lib/Eigen/src/Core/SelfCwiseBinaryOp.h lib/Eigen/src/Core/Dot.h \
 lib/Eigen/src/Core/StableNorm.h lib/Eigen/src/Core/Stride.h \
 lib/Eigen/src/Core/MapBase.h lib/Eigen/src/Core/Map.h \
 lib/Eigen/src/Core/Ref.h lib/Eigen/src/Core/Block.h \
 lib/Eigen/src/Core/VectorBlock.h lib/Eigen/src/Core/IndexedView.h \
 lib/Eigen/src/Core/Transpose.h lib/Eigen/src/Core/DiagonalMatrix.h \
 lib/Eigen/src/Core/Diagonal.h lib/Eigen/src/Core/DiagonalProduct.h \
 lib/Eigen/src/Core/Redux.h lib/Eigen/src/Core/Visitor.h \
 lib/Eigen/src/Core/Fuzzy.h lib/Eigen/src/Core/Swap.h \
 lib/Eigen/src/Core/CommaInitializer.h \
 lib/Eigen/src/Core/GeneralProduct.h lib/Eigen/src/Core/Solve.h \
 lib/Eigen/src/Core/Inverse.h lib/Eigen/src/Core/SolverBase.h \
 lib/Eigen/src/Core/PermutationMatrix.h \
 lib/Eigen/src/Core/Transpositions.h \
 lib/Eigen/src/Core/TriangularMatrix.h \
 lib/Eigen/src/Core/SelfAdjointView.h \
 lib/Eigen/src/Core/products/GeneralBlockPanelKernel.h \
 lib/Eigen/src/Core/products/Parallelizer.h \
 lib/Eigen/src/Core/ProductEvaluators.h \
 lib/Eigen/src/Core/products/GeneralMatrixVector.h \
 lib/Eigen/src/Core/products/GeneralMatrixMatrix.h \
 lib/Eigen/src/Core/SolveTriangular.h \
 lib/Eigen/src/Core/products/GeneralMatrixMatrixTriangular.h \
 lib/Eigen/src/Core/products/SelfadjointMatrixVector.h \
 lib/Eigen/src/Core/products/SelfadjointMatrixMatrix.h \
 lib/Eigen/src/Core/products/SelfadjointProduct.h \
 lib/Eigen/src/Core/products/SelfadjointRank2Update.h \
 lib/Eigen/src/Core/products/TriangularMatrixVector.h \
 lib/Eigen/src/Core/products/TriangularMatrixMatrix.h \
 lib/Eigen/src/Core/products/TriangularSolverMatrix.h \
 lib/Eigen/src/Core/products/TriangularSolverVector.h \
 lib/Eigen/src/Core/BandMatrix.h lib/Eigen/src/Core/CoreIterators.h \
 lib/Eigen/src/Core/ConditionEstimator.h \
 lib/Eigen/src/Core/BooleanRedux.h lib/Eigen/src/Core/Select.h \
 lib/Eigen/src/Core/VectorwiseOp.h lib/Eigen/src/Core/Random.h \
 lib/Eigen/src/Core/Replicate.h lib/Eigen/src/Core/Reverse.h \
 lib/Eigen/src/Core/ArrayWrapper.h lib/Eigen/src/Core/GlobalFunctions.h \
 lib/Eigen/src/Core/util/ReenableStupidWarnings.h lib/Eigen/LU \
 lib/Eigen/src/misc/Kernel.h lib/Eigen/src/misc/Image.h \
 lib/Eigen/src/LU/FullPivLU.h lib/Eigen/src/LU/PartialPivLU.h \
 lib/Eigen/src/LU/Determinant.h lib/Eigen/src/LU/InverseImpl.h \
 lib/Eigen/src/LU/arch/Inverse_SSE.h lib/Eigen/Cholesky \
 lib/Eigen/src/Cholesky/LLT.h lib/Eigen/src/Cholesky/LDLT.h lib/Eigen/QR \
 lib/Eigen/Jacobi lib/Eigen/src/Jacobi/Jacobi.h lib/Eigen/Householder \
 lib/Eigen/src/Householder/Householder.h \
 lib/Eigen/src/Householder/HouseholderSequence.h \
 lib/Eigen/src/Householder/BlockHouseholder.h \
 lib/Eigen/src/QR/HouseholderQR.h lib/Eigen/src/QR/FullPivHouseholderQR.h \
 lib/Eigen/src/QR/ColPivHouseholderQR.h \
 lib/Eigen/src/QR/CompleteOrthogonalDecomposition.h lib/Eigen/SVD \
 lib/Eigen/src/misc/RealSvd2x2.h \
 lib/Eigen/src/SVD/UpperBidiagonalization.h lib/Eigen/src/SVD/SVDBase.h \
 lib/Eigen/src/SVD/JacobiSVD.h lib/Eigen/src/SVD/BDCSVD.h \
 lib/Eigen/Geometry lib/Eigen/src/Geometry/OrthoMethods.h \
 lib/Eigen/src/Geometry/EulerAngles.h \
 lib/Eigen/src/Geometry/Homogeneous.h \
 lib/Eigen/src/Geometry/RotationBase.h \
 lib/Eigen/src/Geometry/Rotation2D.h lib/Eigen/src/Geometry/Quaternion.h \
 lib/Eigen/src/Geometry/AngleAxis.h lib/Eigen/src/Geometry/Transform.h \
 lib/Eigen/src/Geometry/Translation.h lib/Eigen/src/Geometry/Scaling.h \
 lib/Eigen/src/Geometry/Hyperplane.h \
 lib/Eigen/src/Geometry/ParametrizedLine.h \
 lib/Eigen/src/Geometry/AlignedBox.h lib/Eigen/src/Geometry/Umeyama.h \
 lib/Eigen/src/Geometry/arch/Geometry_SSE.h lib/Eigen/Eigenvalues \
 lib/Eigen/src/Eigenvalues/Tridiagonalization.h \
 lib/Eigen/src/Eigenvalues/RealSchur.h \
 lib/Eigen/src/Eigenvalues/./HessenbergDecomposition.h \
 lib/Eigen/src/Eigenvalues/EigenSolver.h \
 lib/Eigen/src/Eigenvalues/./RealSchur.h \
 lib/Eigen/src/Eigenvalues/SelfAdjointEigenSolver.h \
 lib/Eigen/src/Eigenvalues/./Tridiagonalization.h \
 lib/Eigen/src/Eigenvalues/GeneralizedSelfAdjointEigenSolver.h \
 lib/Eigen/src/Eigenvalues/HessenbergDecomposition.h \
 lib/Eigen/src/Eigenvalues/ComplexSchur.h \
 lib/Eigen/src/Eigenvalues/ComplexEigenSolver.h \
 lib/Eigen/src/Eigenvalues/./ComplexSchur.h \
 lib/Eigen/src/Eigenvalues/RealQZ.h \
 lib/Eigen/src/Eigenvalues/GeneralizedEigenSolver.h \
 lib/Eigen/src/Eigenvalues/./RealQZ.h \
 lib/Eigen/src/Eigenvalues/MatrixBaseEigenvalues.h
