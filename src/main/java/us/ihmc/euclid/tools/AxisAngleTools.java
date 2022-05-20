package us.ihmc.euclid.tools;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.axisAngle.interfaces.AxisAngleBasics;
import us.ihmc.euclid.axisAngle.interfaces.AxisAngleReadOnly;
import us.ihmc.euclid.exceptions.NotAMatrix2DException;
import us.ihmc.euclid.matrix.interfaces.Matrix3DBasics;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixBasics;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DBasics;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DReadOnly;
import us.ihmc.euclid.yawPitchRoll.interfaces.YawPitchRollReadOnly;

/**
 * This class provides a collection of static tools to perform operations on axis-angles.
 * <p>
 * This class is mostly used to centralize operations on axis-angles, these operations are available
 * in classes such as {@link AxisAngle} and the user should always using this classes instead of
 * using tools classes. The API of these tools classes is more likely to change over time.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public class AxisAngleTools
{
   /**
    * Tolerance used to identify edge cases, such as when the axis part of an axis-angle can not be
    * normalized.
    */
   public static final double EPS = 1.0e-12;

   private AxisAngleTools()
   {
      // Suppresses default constructor, ensuring non-instantiability.
   }

   /**
    * Transforms the tuple {@code tupleOriginal} using {@code axisAngle} and stores the result in
    * {@code tupleTransformed}.
    * <p>
    * Both tuples can be the same object for performing in place transformation.
    * </p>
    *
    * @param axisAngle        the axis-angle used to transform the tuple. Not modified.
    * @param tupleOriginal    the tuple to transform. Not modified.
    * @param tupleTransformed the tuple in which the result is stored. Modified.
    */
   public static void transform(AxisAngleReadOnly axisAngle, Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      transformImpl(axisAngle, false, tupleOriginal, tupleTransformed);
   }

   /**
    * Performs the inverse of the transform of the tuple {@code tupleOriginal} using {@code axisAngle}
    * and stores the result in {@code tupleTransformed}.
    * <p>
    * This is equivalent to calling
    * {@link #transform(AxisAngleReadOnly, Tuple3DReadOnly, Tuple3DBasics)} with an axis-angle that has
    * an angle of opposite value compared to the given one.
    * </p>
    * <p>
    * Both tuples can be the same object for performing in place transformation.
    * </p>
    *
    * @param axisAngle        the axis-angle used to transform the tuple. Not modified.
    * @param tupleOriginal    the tuple to transform. Not modified.
    * @param tupleTransformed the tuple in which the result is stored. Modified.
    */
   public static void inverseTransform(AxisAngleReadOnly axisAngle, Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      transformImpl(axisAngle, true, tupleOriginal, tupleTransformed);
   }

   /**
    * Transforms the tuple {@code tupleOriginal} using {@code axisAngle} and stores the result in
    * {@code tupleTransformed}.
    * <p>
    * <b> This method is for internal use only. </b>
    * </p>
    * <p>
    * Both tuples can be the same object for performing in place transformation.
    * </p>
    *
    * @param axisAngle        the axis-angle used to transform the tuple. Not modified.
    * @param negateAngle      whether to negate the angle of the axis-angle to perform an inverse
    *                         transform or not.
    * @param tupleOriginal    the tuple to transform. Not modified.
    * @param tupleTransformed the tuple in which the result is stored. Modified.
    */
   private static void transformImpl(AxisAngleReadOnly axisAngle, boolean negateAngle, Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      double angle = axisAngle.getAngle();

      if (negateAngle)
         angle = -angle;

      double cos = EuclidCoreTools.cos(angle);
      double oneMinusCos = 1.0 - cos;
      double sin = EuclidCoreTools.sin(angle);

      double ux = axisAngle.getX();
      double uy = axisAngle.getY();
      double uz = axisAngle.getZ();

      double crossX = uy * tupleOriginal.getZ() - uz * tupleOriginal.getY();
      double crossY = uz * tupleOriginal.getX() - ux * tupleOriginal.getZ();
      double crossZ = ux * tupleOriginal.getY() - uy * tupleOriginal.getX();
      double crossCrossX = uy * crossZ - uz * crossY;
      double crossCrossY = uz * crossX - ux * crossZ;
      double crossCrossZ = ux * crossY - uy * crossX;

      double x = tupleOriginal.getX() + sin * crossX + oneMinusCos * crossCrossX;
      double y = tupleOriginal.getY() + sin * crossY + oneMinusCos * crossCrossY;
      double z = tupleOriginal.getZ() + sin * crossZ + oneMinusCos * crossCrossZ;
      tupleTransformed.set(x, y, z);
   }

   /**
    * Transforms the tuple {@code tupleOriginal} using {@code axisAngle} and adds the result to
    * {@code tupleTransformed}.
    * <p>
    * Both tuples can be the same object for performing in place transformation.
    * </p>
    *
    * @param axisAngle        the axis-angle used to transform the tuple. Not modified.
    * @param tupleOriginal    the tuple to transform. Not modified.
    * @param tupleTransformed the tuple in which the result is stored. Modified.
    */
   public static void addTransform(AxisAngleReadOnly axisAngle, Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      double x = tupleTransformed.getX();
      double y = tupleTransformed.getY();
      double z = tupleTransformed.getZ();
      transform(axisAngle, tupleOriginal, tupleTransformed);
      tupleTransformed.add(x, y, z);
   }

   /**
    * Transforms the tuple {@code tupleOriginal} using {@code axisAngle} and stores the result in
    * {@code tupleTransformed}.
    * <p>
    * Both tuples can be the same object for performing in place transformation.
    * </p>
    *
    * @param axisAngle                 the axis-angle used to transform the tuple. Not modified.
    * @param tupleOriginal             the tuple to transform. Not modified.
    * @param tupleTransformed          the tuple in which the result is stored. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that the axis-angle represents
    *                                  a transformation in the XY plane.
    * @throws NotAMatrix2DException if {@code checkIfTransformInXYPlane == true} and the axis-angle
    *                               does not represent a transformation in the XY plane.
    */
   public static void transform(AxisAngleReadOnly axisAngle, Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed, boolean checkIfTransformInXYPlane)
   {
      transformImpl(axisAngle, false, tupleOriginal, tupleTransformed, checkIfTransformInXYPlane);
   }

   /**
    * Performs the inverse of the transform of the tuple {@code tupleOriginal} using {@code axisAngle}
    * and stores the result in {@code tupleTransformed}.
    * <p>
    * This is equivalent to calling
    * {@link #transform(AxisAngleReadOnly, Tuple2DReadOnly, Tuple2DBasics, boolean)} with an axis-angle
    * that has an angle of opposite value compared to the given one.
    * </p>
    * <p>
    * Both tuples can be the same object for performing in place transformation.
    * </p>
    *
    * @param axisAngle                 the axis-angle used to transform the tuple. Not modified.
    * @param tupleOriginal             the tuple to transform. Not modified.
    * @param tupleTransformed          the tuple in which the result is stored. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that the axis-angle represents
    *                                  a transformation in the XY plane.
    * @throws NotAMatrix2DException if {@code checkIfTransformInXYPlane == true} and the axis-angle
    *                               does not represent a transformation in the XY plane.
    */
   public static void inverseTransform(AxisAngleReadOnly axisAngle,
                                       Tuple2DReadOnly tupleOriginal,
                                       Tuple2DBasics tupleTransformed,
                                       boolean checkIfTransformInXYPlane)
   {
      transformImpl(axisAngle, true, tupleOriginal, tupleTransformed, checkIfTransformInXYPlane);
   }

   /**
    * Transforms the tuple {@code tupleOriginal} using {@code axisAngle} and stores the result in
    * {@code tupleTransformed}.
    * <p>
    * <b> This method is for internal use only. </b>
    * </p>
    * <p>
    * Both tuples can be the same object for performing in place transformation.
    * </p>
    *
    * @param axisAngle                 the axis-angle used to transform the tuple. Not modified.
    * @param negateAngle               whether to negate the angle of the axis-angle to perform an
    *                                  inverse transform or not.
    * @param tupleOriginal             the tuple to transform. Not modified.
    * @param tupleTransformed          the tuple in which the result is stored. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that the axis-angle represents
    *                                  a transformation in the XY plane.
    * @throws NotAMatrix2DException if {@code checkIfTransformInXYPlane == true} and the axis-angle
    */
   private static void transformImpl(AxisAngleReadOnly axisAngle,
                                     boolean negateAngle,
                                     Tuple2DReadOnly tupleOriginal,
                                     Tuple2DBasics tupleTransformed,
                                     boolean checkIfTransformInXYPlane)
   {
      if (checkIfTransformInXYPlane)
         axisAngle.checkIfOrientation2D(EPS);

      double angle = axisAngle.getAngle();

      if (negateAngle)
         angle = -angle;

      double cos = EuclidCoreTools.cos(angle);
      double oneMinusCos = 1.0 - cos;
      double sin = EuclidCoreTools.sin(angle);

      double uz = axisAngle.getZ();

      double crossX = -uz * tupleOriginal.getY();
      double crossY = uz * tupleOriginal.getX();
      double crossCrossX = -uz * crossY;
      double crossCrossY = uz * crossX;

      double x = tupleOriginal.getX() + sin * crossX + oneMinusCos * crossCrossX;
      double y = tupleOriginal.getY() + sin * crossY + oneMinusCos * crossCrossY;
      tupleTransformed.set(x, y);
   }

   /**
    * Transforms the matrix {@code matrixOriginal} using {@code axisAngle} and stores the result in
    * {@code matrixTransformed}.
    * <p>
    * Both matrices can be the same object for performing in place transformation.
    * </p>
    * <p>
    * matrixTransformed = R(axisAngle) * matrixOriginal * R(axisAngle)<sup>-1</sup> <br>
    * where R(axisAngle) is the function to convert an axis-angle into a 3-by-3 rotation matrix.
    * </p>
    *
    * @param axisAngle         the axis-angle used to transform the matrix. Not modified.
    * @param matrixOriginal    the matrix to transform. Not modified.
    * @param matrixTransformed the matrix in which the result is stored. Modified.
    */
   public static void transform(AxisAngleReadOnly axisAngle, Matrix3DReadOnly matrixOriginal, Matrix3DBasics matrixTransformed)
   {
      transformImpl(axisAngle, false, matrixOriginal, matrixTransformed);
   }

   /**
    * Performs the inverse of the transform of the matrix {@code matrixOriginal} using
    * {@code axisAngle} and stores the result in {@code matrixTransformed}.
    * <p>
    * This is equivalent to calling
    * {@link #transform(AxisAngleReadOnly, Matrix3DReadOnly, Matrix3DBasics)} with an axis-angle that
    * has an angle of opposite value compared to the given one.
    * </p>
    * <p>
    * Both matrices can be the same object for performing in place transformation.
    * </p>
    * <p>
    * matrixTransformed = R(axisAngle)<sup>-1</sup> * matrixOriginal * R(axisAngle) <br>
    * where R(axisAngle) is the function to convert an axis-angle into a 3-by-3 rotation matrix.
    * </p>
    *
    * @param axisAngle         the axis-angle used to transform the matrix. Not modified.
    * @param matrixOriginal    the matrix to transform. Not modified.
    * @param matrixTransformed the matrix in which the result is stored. Modified.
    */
   public static void inverseTransform(AxisAngleReadOnly axisAngle, Matrix3DReadOnly matrixOriginal, Matrix3DBasics matrixTransformed)
   {
      transformImpl(axisAngle, true, matrixOriginal, matrixTransformed);
   }

   /**
    * Transforms the matrix {@code matrixOriginal} using {@code axisAngle} and stores the result in
    * {@code matrixTransformed}.
    * <p>
    * <b> This method is for internal use only. </b>
    * </p>
    * <p>
    * <p>
    * Both matrices can be the same object for performing in place transformation.
    * </p>
    * <p>
    * matrixTransformed = R(axisAngle) * matrixOriginal * R(axisAngle)<sup>-1</sup> <br>
    * where R(axisAngle) is the function to convert an axis-angle into a 3-by-3 rotation matrix.
    * </p>
    *
    * @param axisAngle         the axis-angle used to transform the matrix. Not modified.
    * @param negateAngle       whether to negate the angle of the axis-angle to perform an inverse
    *                          transform or not.
    * @param matrixOriginal    the matrix to transform. Not modified.
    * @param matrixTransformed the matrix in which the result is stored. Modified.
    */
   private static void transformImpl(AxisAngleReadOnly axisAngle, boolean negateAngle, Matrix3DReadOnly matrixOriginal, Matrix3DBasics matrixTransformed)
   {
      double cos = EuclidCoreTools.cos(0.5 * axisAngle.getAngle());
      double sin = EuclidCoreTools.sin(0.5 * axisAngle.getAngle());

      double qx = axisAngle.getX() * sin;
      double qy = axisAngle.getY() * sin;
      double qz = axisAngle.getZ() * sin;
      double qs = cos;

      QuaternionTools.transformImpl(qx, qy, qz, qs, negateAngle, matrixOriginal, matrixTransformed);
   }

   /**
    * Transforms the quaternion {@code quaternionOriginal} using {@code axisAngle} and stores the
    * result in {@code quaternionTransformed}.
    * <p>
    * Both {@code quaternionOriginal} and {@code quaternionTransformed} can be the same object for
    * performing in place transformation.
    * </p>
    * <p>
    * Note that this transformation is equivalent to concatenating the orientations of
    * {@code axisAngle} and {@code quaternionOriginal}.
    * </p>
    *
    * @param axisAngle             the axis-angle used to transform the quaternion. Not modified.
    * @param quaternionOriginal    the quaternion to transform. Not modified.
    * @param quaternionTransformed the quaternion in which the result is stored. Modified.
    */
   public static void transform(AxisAngleReadOnly axisAngle, QuaternionReadOnly quaternionOriginal, QuaternionBasics quaternionTransformed)
   {
      QuaternionTools.multiply(axisAngle, false, quaternionOriginal, false, quaternionTransformed);
   }

   /**
    * Performs the inverse of the transform of the quaternion {@code quaternionOriginal} using
    * {@code axisAngle} and stores the result in {@code quaternionTransformed}.
    * <p>
    * This is equivalent to calling
    * {@link #transform(AxisAngleReadOnly, QuaternionReadOnly, QuaternionBasics)} with an axis-angle
    * that has an angle of opposite value compared to the given one.
    * </p>
    * <p>
    * Both {@code quaternionOriginal} and {@code quaternionTransformed} can be the same object for
    * performing in place transformation.
    * </p>
    *
    * @param axisAngle             the axis-angle used to transform the quaternion. Not modified.
    * @param quaternionOriginal    the quaternion to transform. Not modified.
    * @param quaternionTransformed the quaternion in which the result is stored. Modified.
    */
   public static void inverseTransform(AxisAngleReadOnly axisAngle, QuaternionReadOnly quaternionOriginal, QuaternionBasics quaternionTransformed)
   {
      quaternionTransformed.set(quaternionOriginal);
      quaternionTransformed.prependInvertOther(axisAngle);
   }

   /**
    * Transforms the vector {@code vectorOriginal} using {@code axisAngle} and stores the result in
    * {@code vectorTransformed}.
    * <p>
    * Both vectors can be the same object for performing in place transformation.
    * </p>
    *
    * @param axisAngle         the axis-angle used to transform the tuple. Not modified.
    * @param vectorOriginal    the vector to transform. Not modified.
    * @param vectorTransformed the vector in which the result is stored. Modified.
    */
   public static void transform(AxisAngleReadOnly axisAngle, Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
   {
      transformImpl(axisAngle, false, vectorOriginal, vectorTransformed);
   }

   /**
    * Performs the inverse of the transform of the vector {@code vectorOriginal} using
    * {@code axisAngle} and stores the result in {@code vectorTransformed}.
    * <p>
    * This is equivalent to calling
    * {@link #transform(AxisAngleReadOnly, Vector4DReadOnly, Vector4DBasics)} with an axis-angle that
    * has an angle of opposite value compared to the given one.
    * </p>
    * <p>
    * Both vectors can be the same object for performing in place transformation.
    * </p>
    *
    * @param axisAngle         the axis-angle used to transform the tuple. Not modified.
    * @param vectorOriginal    the vector to transform. Not modified.
    * @param vectorTransformed the vector in which the result is stored. Modified.
    */
   public static void inverseTransform(AxisAngleReadOnly axisAngle, Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
   {
      transformImpl(axisAngle, true, vectorOriginal, vectorTransformed);
   }

   /**
    * Transforms the vector {@code vectorOriginal} using {@code axisAngle} and stores the result in
    * {@code vectorTransformed}.
    * <p>
    * <b> This method is for internal use only. </b>
    * </p>
    * <p>
    * Both vectors can be the same object for performing in place transformation.
    * </p>
    *
    * @param axisAngle         the axis-angle used to transform the tuple. Not modified.
    * @param negateAngle       whether to negate the angle of the axis-angle to perform an inverse
    *                          transform or not.
    * @param vectorOriginal    the vector to transform. Not modified.
    * @param vectorTransformed the vector in which the result is stored. Modified.
    */
   private static void transformImpl(AxisAngleReadOnly axisAngle, boolean negateAngle, Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
   {
      double angle = axisAngle.getAngle();

      if (negateAngle)
         angle = -angle;

      double cos = EuclidCoreTools.cos(angle);
      double oneMinusCos = 1.0 - cos;
      double sin = EuclidCoreTools.sin(angle);

      double ux = axisAngle.getX();
      double uy = axisAngle.getY();
      double uz = axisAngle.getZ();

      double crossX = uy * vectorOriginal.getZ() - uz * vectorOriginal.getY();
      double crossY = uz * vectorOriginal.getX() - ux * vectorOriginal.getZ();
      double crossZ = ux * vectorOriginal.getY() - uy * vectorOriginal.getX();
      double crossCrossX = uy * crossZ - uz * crossY;
      double crossCrossY = uz * crossX - ux * crossZ;
      double crossCrossZ = ux * crossY - uy * crossX;

      double x = vectorOriginal.getX() + sin * crossX + oneMinusCos * crossCrossX;
      double y = vectorOriginal.getY() + sin * crossY + oneMinusCos * crossCrossY;
      double z = vectorOriginal.getZ() + sin * crossZ + oneMinusCos * crossCrossZ;
      vectorTransformed.set(x, y, z, vectorOriginal.getS());
   }

   /**
    * Transforms the rotation matrix {@code rotationMatrixOriginal} using {@code axisAngle} and stores
    * the result in {@code rotationMatrixTransformed}.
    * <p>
    * Both rotation matrices can be the same object for performing in place transformation.
    * </p>
    * <p>
    * rotationMatrixTransformed = R(axisAngle) * rotationMatrixOriginal <br>
    * where R(axisAngle) is the function to convert an axis-angle into a 3-by-3 rotation matrix.
    * </p>
    * <p>
    * Note that this transformation is equivalent to concatenating the orientations of
    * {@code axisAngle} and {@code rotationMatrixOriginal}.
    * </p>
    *
    * @param axisAngle                 the axis-angle used to transform the rotation matrix. Not
    *                                  modified.
    * @param rotationMatrixOriginal    the rotation matrix to transform. Not modified.
    * @param rotationMatrixTransformed the rotation matrix in which the result is stored. Modified.
    */
   public static void transform(AxisAngleReadOnly axisAngle, RotationMatrixReadOnly rotationMatrixOriginal, RotationMatrixBasics rotationMatrixTransformed)
   {
      RotationMatrixTools.multiply(axisAngle, false, rotationMatrixOriginal, false, rotationMatrixTransformed);
   }

   /**
    * Performs the inverse of the transform of the rotation matrix {@code rotationMatrixOriginal} using
    * {@code axisAngle} and stores the result in {@code rotationMatrixTransformed}.
    * <p>
    * This is equivalent to calling
    * {@link #transform(AxisAngleReadOnly, RotationMatrixReadOnly, RotationMatrixBasics)} with an
    * axis-angle that has an angle of opposite value compared to the given one.
    * </p>
    * <p>
    * Both rotation matrices can be the same object for performing in place transformation.
    * </p>
    * <p>
    * rotationMatrixTransformed = R(axisAngle)<sup>-1</sup> * rotationMatrixOriginal <br>
    * where R(axisAngle) is the function to convert an axis-angle into a 3-by-3 rotation matrix.
    * </p>
    * <p>
    * Note that this transformation is equivalent to concatenating the orientations of
    * {@code axisAngle} and {@code rotationMatrixOriginal}.
    * </p>
    *
    * @param axisAngle                 the axis-angle used to transform the rotation matrix. Not
    *                                  modified.
    * @param rotationMatrixOriginal    the rotation matrix to transform. Not modified.
    * @param rotationMatrixTransformed the rotation matrix in which the result is stored. Modified.
    */
   public static void inverseTransform(AxisAngleReadOnly axisAngle,
                                       RotationMatrixReadOnly rotationMatrixOriginal,
                                       RotationMatrixBasics rotationMatrixTransformed)
   {
      RotationMatrixTools.multiply(axisAngle, true, rotationMatrixOriginal, false, rotationMatrixTransformed);
   }

   /**
    * Performs the multiplication of {@code aa1} and {@code aa2} and stores the result in
    * {@code axisAngleToPack}.
    * <p>
    * <p>
    * All three arguments can be the same object for in place operations.
    * </p>
    *
    * @param aa1             the first axis-angle in the multiplication. Not modified.
    * @param aa2             the second axis-angle in the multiplication. Not modified.
    * @param axisAngleToPack the axis-angle in which the result is stored. Modified.
    */
   public static void multiply(AxisAngleReadOnly aa1, AxisAngleReadOnly aa2, AxisAngleBasics axisAngleToPack)
   {
      multiplyImpl(aa1, false, aa2, false, axisAngleToPack);
   }

   /**
    * Performs the multiplication of {@code orientation1} and {@code orientation2} and stores the
    * result in {@code axisAngleToPack}.
    * <p>
    * More precisely, {@code orientation1} and {@code orientation2} are first converted to axis-angles,
    * then an axis-angle multiplication is performed using the two first arguments as entry.
    * </p>
    * <p>
    * All three arguments can be the same object for in place operations.
    * </p>
    *
    * @param orientation1    the first orientation in the multiplication. Not modified.
    * @param inverse1        whether the first orientation should be inverted in the multiplication.
    * @param orientation2    the second orientation in the multiplication. Not modified.
    * @param inverse2        whether the second orientation should be inverted in the multiplication.
    * @param axisAngleToPack the axis-angle in which the result is stored. Modified.
    */
   public static void multiply(Orientation3DReadOnly orientation1,
                               boolean inverse1,
                               Orientation3DReadOnly orientation2,
                               boolean inverse2,
                               AxisAngleBasics axisAngleToPack)
   {
      if (orientation1 instanceof AxisAngleReadOnly)
      {
         multiply((AxisAngleReadOnly) orientation1, inverse1, orientation2, inverse2, axisAngleToPack);
         return;
      }

      double beta, u2x, u2y, u2z;
      if (orientation2 instanceof AxisAngleReadOnly)
      { // In this case orientation2 might be the same object as axisAngleToPack, so let's save its components first.
         AxisAngleReadOnly aa2 = (AxisAngleReadOnly) orientation2;
         beta = aa2.getAngle();
         u2x = aa2.getX();
         u2y = aa2.getY();
         u2z = aa2.getZ();
      }
      else
      {
         axisAngleToPack.set(orientation2);
         beta = axisAngleToPack.getAngle();
         u2x = axisAngleToPack.getX();
         u2y = axisAngleToPack.getY();
         u2z = axisAngleToPack.getZ();
      }

      // Now we can safely use the axisAngleToPack argument to convert the orientation1.
      axisAngleToPack.set(orientation1);
      double alpha = axisAngleToPack.getAngle();
      double u1x = axisAngleToPack.getX();
      double u1y = axisAngleToPack.getY();
      double u1z = axisAngleToPack.getZ();
      multiplyImpl(alpha, u1x, u1y, u1z, inverse1, beta, u2x, u2y, u2z, inverse2, axisAngleToPack);
   }

   /**
    * Performs the multiplication of {@code orientation1} and {@code orientation2} and stores the
    * result in {@code axisAngleToPack}.
    * <p>
    * More precisely, {@code orientation1} is first converted to an axis-angle, then an axis-angle
    * multiplication is performed using the two first arguments as entry.
    * </p>
    * <p>
    * All three arguments can be the same object for in place operations.
    * </p>
    *
    * @param orientation1    the first orientation in the multiplication. Not modified.
    * @param inverse1        whether the first orientation should be inverted in the multiplication.
    * @param orientation2    the second orientation in the multiplication. Not modified.
    * @param inverse2        whether the second orientation should be inverted in the multiplication.
    * @param axisAngleToPack the axis-angle in which the result is stored. Modified.
    */
   public static void multiply(Orientation3DReadOnly orientation1,
                               boolean inverse1,
                               AxisAngleReadOnly orientation2,
                               boolean inverse2,
                               AxisAngleBasics axisAngleToPack)
   {
      if (orientation1 instanceof AxisAngleReadOnly)
      {
         multiplyImpl((AxisAngleReadOnly) orientation1, inverse1, orientation2, inverse2, axisAngleToPack);
         return;
      }

      // In this case orientation2 might be the same object as axisAngleToPack, so let's save its components first.
      double beta = orientation2.getAngle();
      double u2x = orientation2.getX();
      double u2y = orientation2.getY();
      double u2z = orientation2.getZ();
      // Now we can safely use the axisAngleToPack argument to convert the orientation1.
      axisAngleToPack.set(orientation1);
      double alpha = axisAngleToPack.getAngle();
      double u1x = axisAngleToPack.getX();
      double u1y = axisAngleToPack.getY();
      double u1z = axisAngleToPack.getZ();
      multiplyImpl(alpha, u1x, u1y, u1z, inverse1, beta, u2x, u2y, u2z, inverse2, axisAngleToPack);
   }

   /**
    * Performs the multiplication of {@code orientation1} and {@code orientation2} and stores the
    * result in {@code axisAngleToPack}.
    * <p>
    * More precisely, {@code orientation2} is first converted to an axis-angle, then an axis-angle
    * multiplication is performed using the two first arguments as entry.
    * </p>
    * <p>
    * All three arguments can be the same object for in place operations.
    * </p>
    *
    * @param orientation1    the first orientation in the multiplication. Not modified.
    * @param inverse1        whether the first orientation should be inverted in the multiplication.
    * @param orientation2    the second orientation in the multiplication. Not modified.
    * @param inverse2        whether the second orientation should be inverted in the multiplication.
    * @param axisAngleToPack the axis-angle in which the result is stored. Modified.
    */
   public static void multiply(AxisAngleReadOnly orientation1,
                               boolean inverse1,
                               Orientation3DReadOnly orientation2,
                               boolean inverse2,
                               AxisAngleBasics axisAngleToPack)
   {
      if (orientation2 instanceof AxisAngleReadOnly)
      {
         multiplyImpl(orientation1, inverse1, (AxisAngleReadOnly) orientation2, inverse2, axisAngleToPack);
         return;
      }

      // In this case orientation1 might be the same object as axisAngleToPack, so let's save its components first.
      double alpha = orientation1.getAngle();
      double u1x = orientation1.getX();
      double u1y = orientation1.getY();
      double u1z = orientation1.getZ();
      // Now we can safely use the axisAngleToPack argument to convert the orientation2.
      axisAngleToPack.set(orientation2);
      double beta = axisAngleToPack.getAngle();
      double u2x = axisAngleToPack.getX();
      double u2y = axisAngleToPack.getY();
      double u2z = axisAngleToPack.getZ();
      multiplyImpl(alpha, u1x, u1y, u1z, inverse1, beta, u2x, u2y, u2z, inverse2, axisAngleToPack);
   }

   /**
    * Performs the multiplication of the inverse of {@code aa1} and {@code aa2} and stores the result
    * in {@code axisAngleToPack}.
    * <p>
    * <p>
    * All three arguments can be the same object for in place operations.
    * </p>
    *
    * @param aa1             the first axis-angle in the multiplication. Not modified.
    * @param aa2             the second axis-angle in the multiplication. Not modified.
    * @param axisAngleToPack the axis-angle in which the result is stored. Modified.
    */
   public static void multiplyInvertLeft(AxisAngleReadOnly aa1, AxisAngleReadOnly aa2, AxisAngleBasics axisAngleToPack)
   {
      multiplyImpl(aa1, true, aa2, false, axisAngleToPack);
   }

   /**
    * Performs the multiplication of {@code aa1} and the inverse of {@code aa2} and stores the result
    * in {@code axisAngleToPack}.
    * <p>
    * <p>
    * All three arguments can be the same object for in place operations.
    * </p>
    *
    * @param aa1             the first axis-angle in the multiplication. Not modified.
    * @param aa2             the second axis-angle in the multiplication. Not modified.
    * @param axisAngleToPack the axis-angle in which the result is stored. Modified.
    */
   public static void multiplyInvertRight(AxisAngleReadOnly aa1, AxisAngleReadOnly aa2, AxisAngleBasics axisAngleToPack)
   {
      multiplyImpl(aa1, false, aa2, true, axisAngleToPack);
   }

   /**
    * Performs the multiplication of the inverse of {@code aa1} and the inverse of {@code aa2} and
    * stores the result in {@code axisAngleToPack}.
    * <p>
    * <p>
    * All three arguments can be the same object for in place operations.
    * </p>
    *
    * @param aa1             the first axis-angle in the multiplication. Not modified.
    * @param aa2             the second axis-angle in the multiplication. Not modified.
    * @param axisAngleToPack the axis-angle in which the result is stored. Modified.
    */
   public static void multiplyInvertBoth(AxisAngleReadOnly aa1, AxisAngleReadOnly aa2, AxisAngleBasics axisAngleToPack)
   {
      multiplyImpl(aa1, true, aa2, true, axisAngleToPack);
   }

   /**
    * Performs the multiplication of {@code aa1} and {@code aa2} and stores the result in
    * {@code axisAngleToPack}.
    * <p>
    * <b> This method is for internal use only. </b>
    * </p>
    * <p>
    * Provides the option to invert either axis-angle when multiplying them.
    * <p>
    * All three arguments can be the same object for in place operations.
    * </p>
    * <p>
    * <a href="https://i.imgur.com/Mdc2AV3.jpg"> Useful link</a>
    * </p>
    *
    * @param aa1             the first axis-angle in the multiplication. Not modified.
    * @param inverse1        whether to inverse {@code aa1} or not.
    * @param aa2             the second axis-angle in the multiplication. Not modified.
    * @param inverse2        whether to inverse {@code aa2} or not.
    * @param axisAngleToPack the axis-angle in which the result is stored. Modified.
    */
   private static void multiplyImpl(AxisAngleReadOnly aa1, boolean inverse1, AxisAngleReadOnly aa2, boolean inverse2, AxisAngleBasics axisAngleToPack)
   {
      double alpha = aa1.getAngle();
      double u1x = aa1.getX();
      double u1y = aa1.getY();
      double u1z = aa1.getZ();
      double beta = aa2.getAngle();
      double u2x = aa2.getX();
      double u2y = aa2.getY();
      double u2z = aa2.getZ();

      multiplyImpl(alpha, u1x, u1y, u1z, inverse1, beta, u2x, u2y, u2z, inverse2, axisAngleToPack);
   }

   private static void multiplyImpl(double alpha,
                                    double u1x,
                                    double u1y,
                                    double u1z,
                                    boolean inverse1,
                                    double beta,
                                    double u2x,
                                    double u2y,
                                    double u2z,
                                    boolean inverse2,
                                    AxisAngleBasics axisAngleToPack)
   {
      double axisNorm1 = EuclidCoreTools.fastNorm(u1x, u1y, u1z);
      if (axisNorm1 < EPS)
         return;

      double axisNorm2 = EuclidCoreTools.fastNorm(u2x, u2y, u2z);
      if (axisNorm2 < EPS)
         return;

      axisNorm1 = 1.0 / axisNorm1;

      if (inverse1)
         alpha = -alpha;
      u1x *= axisNorm1;
      u1y *= axisNorm1;
      u1z *= axisNorm1;

      axisNorm2 = 1.0 / axisNorm2;

      if (inverse2)
         beta = -beta;
      u2x *= axisNorm2;
      u2y *= axisNorm2;
      u2z *= axisNorm2;

      double cosHalfAlpha = EuclidCoreTools.cos(0.5 * alpha);
      double sinHalfAlpha = EuclidCoreTools.sin(0.5 * alpha);
      double cosHalfBeta = EuclidCoreTools.cos(0.5 * beta);
      double sinHalfBeta = EuclidCoreTools.sin(0.5 * beta);

      double dot = u1x * u2x + u1y * u2y + u1z * u2z;
      double crossX = u1y * u2z - u1z * u2y;
      double crossY = u1z * u2x - u1x * u2z;
      double crossZ = u1x * u2y - u1y * u2x;

      double sinCos = sinHalfAlpha * cosHalfBeta;
      double cosSin = cosHalfAlpha * sinHalfBeta;
      double cosCos = cosHalfAlpha * cosHalfBeta;
      double sinSin = sinHalfAlpha * sinHalfBeta;

      double cosHalfGamma = cosCos - sinSin * dot;

      double sinHalfGammaUx = sinCos * u1x + cosSin * u2x + sinSin * crossX;
      double sinHalfGammaUy = sinCos * u1y + cosSin * u2y + sinSin * crossY;
      double sinHalfGammaUz = sinCos * u1z + cosSin * u2z + sinSin * crossZ;

      double sinHalfGammaSquared = EuclidCoreTools.normSquared(sinHalfGammaUx, sinHalfGammaUy, sinHalfGammaUz);

      if (sinHalfGammaSquared < EPS)
      {
         axisAngleToPack.set(1.0, 0.0, 0.0, 0.0);
      }
      else
      {
         double sinHalfGamma = EuclidCoreTools.squareRoot(sinHalfGammaSquared);

         double gamma = 2.0 * EuclidCoreTools.atan2(sinHalfGamma, cosHalfGamma);
         double sinHalfGammaInv = 1.0 / sinHalfGamma;
         double ux = sinHalfGammaUx * sinHalfGammaInv;
         double uy = sinHalfGammaUy * sinHalfGammaInv;
         double uz = sinHalfGammaUz * sinHalfGammaInv;
         axisAngleToPack.set(ux, uy, uz, gamma);
      }
   }

   /**
    * Prepend a rotation about the z-axis to {@code axisAngleOriginal} and stores the result in
    * {@code axisAngleToPack}.
    * <p>
    * All the axis-angles can be the same object.
    * </p>
    *
    * <pre>
    *                   / ux    =  0  \
    * axisAngleToPack = | uy    =  0  | * axisAngleOriginal
    *                   | uz    =  1  |
    *                   \ angle = yaw /
    * </pre>
    *
    * @param yaw               the angle to rotate about the z-axis.
    * @param axisAngleOriginal the axis-angle on which the yaw rotation is prepended. Not modified.
    * @param axisAngleToPack   the axis-angle in which the result is stored. Modified.
    */
   public static void prependYawRotation(double yaw, AxisAngleReadOnly axisAngleOriginal, AxisAngleBasics axisAngleToPack)
   {
      double beta = axisAngleOriginal.getAngle();
      double ux = axisAngleOriginal.getX();
      double uy = axisAngleOriginal.getY();
      double uz = axisAngleOriginal.getZ();

      double cosHalfAlpha = EuclidCoreTools.cos(0.5 * yaw);
      double sinHalfAlpha = EuclidCoreTools.sin(0.5 * yaw);
      double cosHalfBeta = EuclidCoreTools.cos(0.5 * beta);
      double sinHalfBeta = EuclidCoreTools.sin(0.5 * beta);

      double sinCos = sinHalfAlpha * cosHalfBeta;
      double cosSin = cosHalfAlpha * sinHalfBeta;
      double cosCos = cosHalfAlpha * cosHalfBeta;
      double sinSin = sinHalfAlpha * sinHalfBeta;

      double cosHalfGamma = cosCos - sinSin * uz;

      double sinHalfGammaUx = cosSin * ux - sinSin * uy;
      double sinHalfGammaUy = cosSin * uy + sinSin * ux;
      double sinHalfGammaUz = sinCos + cosSin * uz;

      double sinHalfGamma = EuclidCoreTools.norm(sinHalfGammaUx, sinHalfGammaUy, sinHalfGammaUz);

      double gamma = 2.0 * EuclidCoreTools.atan2(sinHalfGamma, cosHalfGamma);
      double sinHalfGammaInv = 1.0 / sinHalfGamma;
      axisAngleToPack.set(sinHalfGammaUx * sinHalfGammaInv, sinHalfGammaUy * sinHalfGammaInv, sinHalfGammaUz * sinHalfGammaInv, gamma);
   }

   /**
    * Append a rotation about the z-axis to {@code axisAngleOriginal} and stores the result in
    * {@code axisAngleToPack}.
    * <p>
    * All the axis-angles can be the same object.
    * </p>
    *
    * <pre>
    *                                       / ux    =  0  \
    * axisAngleToPack = axisAngleOriginal * | uy    =  0  |
    *                                       | uz    =  1  |
    *                                       \ angle = yaw /
    * </pre>
    *
    * @param axisAngleOriginal the axis-angle on which the yaw rotation is appended. Not modified.
    * @param yaw               the angle to rotate about the z-axis.
    * @param axisAngleToPack   the axis-angle in which the result is stored. Modified.
    */
   public static void appendYawRotation(AxisAngleReadOnly axisAngleOriginal, double yaw, AxisAngleBasics axisAngleToPack)
   {
      double alpha = axisAngleOriginal.getAngle();
      double ux = axisAngleOriginal.getX();
      double uy = axisAngleOriginal.getY();
      double uz = axisAngleOriginal.getZ();

      double cosHalfAlpha = EuclidCoreTools.cos(0.5 * alpha);
      double sinHalfAlpha = EuclidCoreTools.sin(0.5 * alpha);
      double cosHalfBeta = EuclidCoreTools.cos(0.5 * yaw);
      double sinHalfBeta = EuclidCoreTools.sin(0.5 * yaw);

      double sinCos = sinHalfAlpha * cosHalfBeta;
      double cosSin = cosHalfAlpha * sinHalfBeta;
      double cosCos = cosHalfAlpha * cosHalfBeta;
      double sinSin = sinHalfAlpha * sinHalfBeta;

      double cosHalfGamma = cosCos - sinSin * uz;

      double sinHalfGammaUx = sinCos * ux + sinSin * uy;
      double sinHalfGammaUy = sinCos * uy - sinSin * ux;
      double sinHalfGammaUz = sinCos * uz + cosSin;

      double sinHalfGamma = EuclidCoreTools.norm(sinHalfGammaUx, sinHalfGammaUy, sinHalfGammaUz);

      double gamma = 2.0 * EuclidCoreTools.atan2(sinHalfGamma, cosHalfGamma);
      double sinHalfGammaInv = 1.0 / sinHalfGamma;
      axisAngleToPack.set(sinHalfGammaUx * sinHalfGammaInv, sinHalfGammaUy * sinHalfGammaInv, sinHalfGammaUz * sinHalfGammaInv, gamma);
   }

   /**
    * Prepend a rotation about the y-axis to {@code axisAngleOriginal} and stores the result in
    * {@code axisAngleToPack}.
    * <p>
    * All the axis-angles can be the same object.
    * </p>
    *
    * <pre>
    *                   / ux    =  0    \
    * axisAngleToPack = | uy    =  1    | * axisAngleOriginal
    *                   | uz    =  0    |
    *                   \ angle = pitch /
    * </pre>
    *
    * @param pitch             the angle to rotate about the y-axis.
    * @param axisAngleOriginal the axis-angle on which the yaw rotation is prepended. Not modified.
    * @param axisAngleToPack   the axis-angle in which the result is stored. Modified.
    */
   public static void prependPitchRotation(double pitch, AxisAngleReadOnly axisAngleOriginal, AxisAngleBasics axisAngleToPack)
   {
      double beta = axisAngleOriginal.getAngle();
      double ux = axisAngleOriginal.getX();
      double uy = axisAngleOriginal.getY();
      double uz = axisAngleOriginal.getZ();

      double cosHalfAlpha = EuclidCoreTools.cos(0.5 * pitch);
      double sinHalfAlpha = EuclidCoreTools.sin(0.5 * pitch);
      double cosHalfBeta = EuclidCoreTools.cos(0.5 * beta);
      double sinHalfBeta = EuclidCoreTools.sin(0.5 * beta);

      double sinCos = sinHalfAlpha * cosHalfBeta;
      double cosSin = cosHalfAlpha * sinHalfBeta;
      double cosCos = cosHalfAlpha * cosHalfBeta;
      double sinSin = sinHalfAlpha * sinHalfBeta;

      double cosHalfGamma = cosCos - sinSin * uy;

      double sinHalfGammaUx = cosSin * ux + sinSin * uz;
      double sinHalfGammaUy = sinCos + cosSin * uy;
      double sinHalfGammaUz = cosSin * uz - sinSin * ux;

      double sinHalfGamma = EuclidCoreTools.norm(sinHalfGammaUx, sinHalfGammaUy, sinHalfGammaUz);

      double gamma = 2.0 * EuclidCoreTools.atan2(sinHalfGamma, cosHalfGamma);
      double sinHalfGammaInv = 1.0 / sinHalfGamma;
      axisAngleToPack.set(sinHalfGammaUx * sinHalfGammaInv, sinHalfGammaUy * sinHalfGammaInv, sinHalfGammaUz * sinHalfGammaInv, gamma);
   }

   /**
    * Append a rotation about the y-axis to {@code axisAngleOriginal} and stores the result in
    * {@code axisAngleToPack}.
    * <p>
    * All the axis-angles can be the same object.
    * </p>
    *
    * <pre>
    *                                       / ux    =  0    \
    * axisAngleToPack = axisAngleOriginal * | uy    =  1    |
    *                                       | uz    =  0    |
    *                                       \ angle = pitch /
    * </pre>
    *
    * @param axisAngleOriginal the axis-angle on which the yaw rotation is appended. Not modified.
    * @param pitch             the angle to rotate about the y-axis.
    * @param axisAngleToPack   the axis-angle in which the result is stored. Modified.
    */
   public static void appendPitchRotation(AxisAngleReadOnly axisAngleOriginal, double pitch, AxisAngleBasics axisAngleToPack)
   {
      double alpha = axisAngleOriginal.getAngle();
      double ux = axisAngleOriginal.getX();
      double uy = axisAngleOriginal.getY();
      double uz = axisAngleOriginal.getZ();

      double cosHalfAlpha = EuclidCoreTools.cos(0.5 * alpha);
      double sinHalfAlpha = EuclidCoreTools.sin(0.5 * alpha);
      double cosHalfBeta = EuclidCoreTools.cos(0.5 * pitch);
      double sinHalfBeta = EuclidCoreTools.sin(0.5 * pitch);

      double sinCos = sinHalfAlpha * cosHalfBeta;
      double cosSin = cosHalfAlpha * sinHalfBeta;
      double cosCos = cosHalfAlpha * cosHalfBeta;
      double sinSin = sinHalfAlpha * sinHalfBeta;

      double cosHalfGamma = cosCos - sinSin * uy;

      double sinHalfGammaUx = sinCos * ux - sinSin * uz;
      double sinHalfGammaUy = sinCos * uy + cosSin;
      double sinHalfGammaUz = sinCos * uz + sinSin * ux;

      double sinHalfGamma = EuclidCoreTools.norm(sinHalfGammaUx, sinHalfGammaUy, sinHalfGammaUz);

      double gamma = 2.0 * EuclidCoreTools.atan2(sinHalfGamma, cosHalfGamma);
      double sinHalfGammaInv = 1.0 / sinHalfGamma;
      axisAngleToPack.set(sinHalfGammaUx * sinHalfGammaInv, sinHalfGammaUy * sinHalfGammaInv, sinHalfGammaUz * sinHalfGammaInv, gamma);
   }

   /**
    * Prepend a rotation about the x-axis to {@code axisAngleOriginal} and stores the result in
    * {@code axisAngleToPack}.
    * <p>
    * All the axis-angles can be the same object.
    * </p>
    *
    * <pre>
    *                   / ux    =  1   \
    * axisAngleToPack = | uy    =  0   | * axisAngleOriginal
    *                   | uz    =  0   |
    *                   \ angle = roll /
    * </pre>
    *
    * @param roll              the angle to rotate about the x-axis.
    * @param axisAngleOriginal the axis-angle on which the yaw rotation is prepended. Not modified.
    * @param axisAngleToPack   the axis-angle in which the result is stored. Modified.
    */
   public static void prependRollRotation(double roll, AxisAngleReadOnly axisAngleOriginal, AxisAngleBasics axisAngleToPack)
   {
      double beta = axisAngleOriginal.getAngle();
      double ux = axisAngleOriginal.getX();
      double uy = axisAngleOriginal.getY();
      double uz = axisAngleOriginal.getZ();

      double cosHalfAlpha = EuclidCoreTools.cos(0.5 * roll);
      double sinHalfAlpha = EuclidCoreTools.sin(0.5 * roll);
      double cosHalfBeta = EuclidCoreTools.cos(0.5 * beta);
      double sinHalfBeta = EuclidCoreTools.sin(0.5 * beta);

      double sinCos = sinHalfAlpha * cosHalfBeta;
      double cosSin = cosHalfAlpha * sinHalfBeta;
      double cosCos = cosHalfAlpha * cosHalfBeta;
      double sinSin = sinHalfAlpha * sinHalfBeta;

      double cosHalfGamma = cosCos - sinSin * ux;

      double sinHalfGammaUx = sinCos + cosSin * ux;
      double sinHalfGammaUy = cosSin * uy - sinSin * uz;
      double sinHalfGammaUz = cosSin * uz + sinSin * uy;

      double sinHalfGamma = EuclidCoreTools.norm(sinHalfGammaUx, sinHalfGammaUy, sinHalfGammaUz);

      double gamma = 2.0 * EuclidCoreTools.atan2(sinHalfGamma, cosHalfGamma);
      double sinHalfGammaInv = 1.0 / sinHalfGamma;
      axisAngleToPack.set(sinHalfGammaUx * sinHalfGammaInv, sinHalfGammaUy * sinHalfGammaInv, sinHalfGammaUz * sinHalfGammaInv, gamma);
   }

   /**
    * Append a rotation about the x-axis to {@code axisAngleOriginal} and stores the result in
    * {@code axisAngleToPack}.
    * <p>
    * All the axis-angles can be the same object.
    * </p>
    *
    * <pre>
    *                                       / ux    =  1   \
    * axisAngleToPack = axisAngleOriginal * | uy    =  0   |
    *                                       | uz    =  0   |
    *                                       \ angle = roll /
    * </pre>
    *
    * @param axisAngleOriginal the axis-angle on which the yaw rotation is appended. Not modified.
    * @param roll              the angle to rotate about the x-axis.
    * @param axisAngleToPack   the axis-angle in which the result is stored. Modified.
    */
   public static void appendRollRotation(AxisAngleReadOnly axisAngleOriginal, double roll, AxisAngleBasics axisAngleToPack)
   {
      double alpha = axisAngleOriginal.getAngle();
      double ux = axisAngleOriginal.getX();
      double uy = axisAngleOriginal.getY();
      double uz = axisAngleOriginal.getZ();

      double cosHalfAlpha = EuclidCoreTools.cos(0.5 * alpha);
      double sinHalfAlpha = EuclidCoreTools.sin(0.5 * alpha);
      double cosHalfBeta = EuclidCoreTools.cos(0.5 * roll);
      double sinHalfBeta = EuclidCoreTools.sin(0.5 * roll);

      double sinCos = sinHalfAlpha * cosHalfBeta;
      double cosSin = cosHalfAlpha * sinHalfBeta;
      double cosCos = cosHalfAlpha * cosHalfBeta;
      double sinSin = sinHalfAlpha * sinHalfBeta;

      double cosHalfGamma = cosCos - sinSin * ux;

      double sinHalfGammaUx = sinCos * ux + cosSin;
      double sinHalfGammaUy = sinCos * uy + sinSin * uz;
      double sinHalfGammaUz = sinCos * uz + -sinSin * uy;

      double sinHalfGamma = EuclidCoreTools.norm(sinHalfGammaUx, sinHalfGammaUy, sinHalfGammaUz);

      double gamma = 2.0 * EuclidCoreTools.atan2(sinHalfGamma, cosHalfGamma);
      double sinHalfGammaInv = 1.0 / sinHalfGamma;
      axisAngleToPack.set(sinHalfGammaUx * sinHalfGammaInv, sinHalfGammaUy * sinHalfGammaInv, sinHalfGammaUz * sinHalfGammaInv, gamma);
   }
   
   /**
    * Performs a Cross platform Angular Distance Calculation between Axis Angle and any other 3D orientation systems. 
    * @param axisAngle
    * @param orientation3D
    * @param limitToPi    converts the resulting angular distance to within [0 , <i>pi</i>] if set true.
    */
   public static double distance(AxisAngleReadOnly axisAngle, Orientation3DReadOnly orientation3D, boolean limitToPi)
   {
      if (orientation3D instanceof QuaternionReadOnly)
      {
         return distance(axisAngle, (QuaternionReadOnly) orientation3D , limitToPi);
      }
      if (orientation3D instanceof YawPitchRollReadOnly)
      {
         return distance(axisAngle, (YawPitchRollReadOnly) orientation3D, limitToPi);
      }
      if (orientation3D instanceof AxisAngleReadOnly)
      {
         return distance(axisAngle, (AxisAngleReadOnly) orientation3D, limitToPi);
      }
      if (orientation3D instanceof RotationMatrixReadOnly)
      {
         return distance(axisAngle, (RotationMatrixReadOnly) orientation3D);
      }
      else
      {
         throw new UnsupportedOperationException("Unsupported type: " + orientation3D.getClass().getSimpleName());
      }
   }
   /**
    * Computes and returns Angular Distance between Axis Angle and Quaternion. 
    * @param axisAngle the axisAngle to be used for comparison. Not modified
    * @param quaternion the quaternion to be used for comparison. Not modified
    * @param limitToPi limits the result to [0 , <i>pi</i>] if set true.
    * @return angular distance between the two orientations in range: [0, 2<i>pi</i>]
    */
   public static double distance(AxisAngleReadOnly axisAngle, QuaternionReadOnly quaternion, boolean limitToPi)
   {

      if (axisAngle.containsNaN() || quaternion.containsNaN())
      {
         return Double.NaN;
      }
      if (axisAngle.isZeroOrientation(EPS))
      {
         return QuaternionTools.angle(quaternion);
      }
      if (quaternion.isZeroOrientation(EPS))
      {
         return axisAngle.getAngle();
      }
      
      // Converting self(AxisAngle) to quaternion.
      double ux = axisAngle.getX();
      double uy = axisAngle.getY();
      double uz = axisAngle.getZ();
      double angle = axisAngle.getAngle();
      double qs, qx, qy, qz;
      double uNorm = EuclidCoreTools.fastNorm(ux, uy, uz);
      if (uNorm < EPS)
      {
         return QuaternionTools.angle(quaternion);
      }
      else
      {
         double halfTheta = 0.5 * angle;
         double cosHalfTheta = EuclidCoreTools.cos(halfTheta);
         double sinHalfTheta = EuclidCoreTools.sin(halfTheta) / uNorm;
         qx = ux * sinHalfTheta;
         qy = uy * sinHalfTheta;
         qz = uz * sinHalfTheta;
         qs = cosHalfTheta;
      }

      return QuaternionTools.distance(quaternion.getX(), quaternion.getY(), quaternion.getZ(), quaternion.getS(), qx, qy, qz, qs, limitToPi);
   }

   /**
    * Computes and returns Angular Distance between Axis Angle and Rotation Matrix. 
    * @param axisAngle the axisAngle to be used for comparison. Not modified
    * @param rotationMatrix the rotationMatrix to be used for comparison. Not modified
    * @return angular distance between the two orientations in range: [0, <i>pi</i>]
    */
   public static double distance(AxisAngleReadOnly axisAngle, RotationMatrixReadOnly rotationMatrix)
   {
      if (axisAngle.containsNaN() || rotationMatrix.containsNaN())
      {
         return Double.NaN;
      }
      if (axisAngle.isZeroOrientation(EPS))
      {
         return RotationMatrixTools.angle(rotationMatrix);
      }
      if (rotationMatrix.isZeroOrientation(EPS))
      {
         return axisAngle.getAngle();
      }
      double ux = axisAngle.getX();
      double uy = axisAngle.getY();
      double uz = axisAngle.getZ();
      double angle = axisAngle.getAngle();
      double m00 = 0, m01 = 0, m02 = 0, m10 = 0, m11 = 0, m12 = 0, m20 = 0, m21 = 0, m22 = 0;

      double uNorm = EuclidCoreTools.fastNorm(ux, uy, uz);
      if (uNorm < EPS)
      {
         return RotationMatrixTools.angle(rotationMatrix);
      }
      else
      {
         uNorm = 1.0 / uNorm;
         double ax = ux * uNorm;
         double ay = uy * uNorm;
         double az = uz * uNorm;

         double sinTheta = EuclidCoreTools.sin(angle);
         double cosTheta = EuclidCoreTools.cos(angle);
         double t = 1.0 - cosTheta;

         double xz = ax * az;
         double xy = ax * ay;
         double yz = ay * az;

         m00 = t * ax * ax + cosTheta;
         m01 = t * xy - sinTheta * az;
         m02 = t * xz + sinTheta * ay;
         m10 = t * xy + sinTheta * az;
         m11 = t * ay * ay + cosTheta;
         m12 = t * yz - sinTheta * ax;
         m20 = t * xz - sinTheta * ay;
         m21 = t * yz + sinTheta * ax;
         m22 = t * az * az + cosTheta;
      }
      return RotationMatrixTools.distance(rotationMatrix, m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }
   /**
    * Computes and returns Angular Distance between Axis Angle and yawPitchRoll. 
    * @param axisAngle the axisAngle to be used for comparison. Not modified
    * @param yawPitchRoll the yawPitchRoll to be used for comparison. Not modified
    * @return angular distance between the two orientations in range: [0, 2<i>pi</i>]
    */
   public static double distance(AxisAngleReadOnly axisAngle, YawPitchRollReadOnly yawPitchRoll, boolean limitToPi)
   {
      if (axisAngle.containsNaN() || axisAngle.containsNaN())
      {
         return Double.NaN;
      }
      if (axisAngle.isZeroOrientation(EPS))
      {
         return YawPitchRollTools.angle(yawPitchRoll);
      }
      
      if (yawPitchRoll.isZeroOrientation(EPS))
      {
         return axisAngle.getAngle();
      }
      double yaw = yawPitchRoll.getYaw();
      double pitch = yawPitchRoll.getPitch();
      double roll = yawPitchRoll.getRoll();
      double angle, ax, ay, az;



      double halfYaw = yaw / 2.0;
      double cYaw = EuclidCoreTools.cos(halfYaw);
      double sYaw = EuclidCoreTools.sin(halfYaw);

      double halfPitch = pitch / 2.0;
      double cPitch = EuclidCoreTools.cos(halfPitch);
      double sPitch = EuclidCoreTools.sin(halfPitch);

      double halfRoll = roll / 2.0;
      double cRoll = EuclidCoreTools.cos(halfRoll);
      double sRoll = EuclidCoreTools.sin(halfRoll);

      double qs = cYaw * cPitch * cRoll + sYaw * sPitch * sRoll;
      double qx = cYaw * cPitch * sRoll - sYaw * sPitch * cRoll;
      double qy = sYaw * cPitch * sRoll + cYaw * sPitch * cRoll;
      double qz = sYaw * cPitch * cRoll - cYaw * sPitch * sRoll;

      double uNorm = EuclidCoreTools.norm(qx, qy, qz);

      if (uNorm > EPS)
      {
         angle = 2.0 * EuclidCoreTools.atan2(uNorm, qs);
         uNorm = 1.0 / uNorm;
         ax = qx * uNorm;
         ay = qy * uNorm;
         az = qz * uNorm;
      }
      else
      {
         return axisAngle.getAngle();
      }
      return distance(axisAngle, ax, ay, az, angle, limitToPi);
   }

   /**
    * Computes and returns the distance between the two axis-angles {@code aa1} and {@code aa2}.
    *
    * @param aa1 the first axis-angle to measure the distance. Not modified.
    * @param aa2 the second axis-angle to measure the distance. Not modified.
    * @return the angle representing the distance between the two axis-angles. It is contained in [0,
    *         2<i>pi</i>]
    */
   public static double distance(AxisAngleReadOnly aa1, AxisAngleReadOnly aa2, boolean limitToPi)
   {
      return distance(aa1, aa2.getX(), aa2.getY(), aa2.getZ(), aa2.getAngle(), limitToPi);
   }

   static double distance(AxisAngleReadOnly aa1, double u2x, double u2y, double u2z, double u2a, boolean limitToPi)
   {
      double alpha = aa1.getAngle();
      double u1x = aa1.getX();
      double u1y = aa1.getY();
      double u1z = aa1.getZ();

      double beta = -u2a;

      double cosHalfAlpha = EuclidCoreTools.cos(0.5 * alpha);
      double sinHalfAlpha = EuclidCoreTools.sin(0.5 * alpha);
      double cosHalfBeta = EuclidCoreTools.cos(0.5 * beta);
      double sinHalfBeta = EuclidCoreTools.sin(0.5 * beta);

      double dot = u1x * u2x + u1y * u2y + u1z * u2z;
      double crossX = u1y * u2z - u1z * u2y;
      double crossY = u1z * u2x - u1x * u2z;
      double crossZ = u1x * u2y - u1y * u2x;

      double sinCos = sinHalfAlpha * cosHalfBeta;
      double cosSin = cosHalfAlpha * sinHalfBeta;
      double cosCos = cosHalfAlpha * cosHalfBeta;
      double sinSin = sinHalfAlpha * sinHalfBeta;

      double cosHalfGamma = cosCos - sinSin * dot;

      double sinHalfGammaUx = sinCos * u1x + cosSin * u2x + sinSin * crossX;
      double sinHalfGammaUy = sinCos * u1y + cosSin * u2y + sinSin * crossY;
      double sinHalfGammaUz = sinCos * u1z + cosSin * u2z + sinSin * crossZ;

      double sinHalfGamma = EuclidCoreTools.norm(sinHalfGammaUx, sinHalfGammaUy, sinHalfGammaUz);

      double gamma = 2.0 * EuclidCoreTools.atan2(sinHalfGamma, cosHalfGamma);
      
      if(limitToPi && gamma > Math.PI)
      {
         gamma = 2*Math.PI - gamma;
      }
      return Math.abs(gamma);
   }
}
