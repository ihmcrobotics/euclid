package us.ihmc.euclid.tools;

import us.ihmc.euclid.exceptions.NotAMatrix2DException;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.Matrix3DBasics;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DBasics;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.*;

/**
 * This gathers common mathematical operations involving quaternions.
 *
 * @author Sylvain Bertrand
 */
public class QuaternionTools
{
   static final double EPS = 1.0e-12;

   private QuaternionTools()
   {
      // Suppresses default constructor, ensuring non-instantiability.
   }

   /**
    * Tests that the given {@code quaternion} is equal to the neutral quaternion on a per-component
    * basis.
    *
    * @param quaternion the query. Not modified.
    * @param epsilon    the tolerance used for the comparison.
    * @return {@code true} if the quaternion equal to the neutral quaternion, {@code false} otherwise.
    */
   public static boolean isNeutralQuaternion(QuaternionReadOnly quaternion, double epsilon)
   {
      return isNeutralQuaternion(quaternion.getX(), quaternion.getY(), quaternion.getZ(), quaternion.getS(), epsilon);
   }

   /**
    * Tests that the given quaternion is equal to the neutral quaternion on a per-component basis.
    *
    * @param qx      the x-component of the quaternion. Not modified.
    * @param qy      the y-component of the quaternion. Not modified.
    * @param qz      the z-component of the quaternion. Not modified.
    * @param qs      the s-component of the quaternion. Not modified.
    * @param epsilon the tolerance used for the comparison.
    * @return {@code true} if the quaternion equal to the neutral quaternion, {@code false} otherwise.
    */
   public static boolean isNeutralQuaternion(double qx, double qy, double qz, double qs, double epsilon)
   {
      return EuclidCoreTools.epsilonEquals(1.0, qs, epsilon) && Math.abs(qx) <= epsilon && Math.abs(qy) <= epsilon && Math.abs(qz) <= epsilon;
   }

   /**
    * Performs the multiplication of {@code q1} and {@code q2} and stores the result in
    * {@code quaternionToPack}.
    * <p>
    * quaternionToPack = q1 * q2
    * </p>
    * <p>
    * All three arguments can be the same object for in place operations.
    * </p>
    *
    * @param q1               the first quaternion in the multiplication. Not modified.
    * @param q2               the second quaternion in the multiplication. Not modified.
    * @param quaternionToPack the quaternion in which the result is stores. Modified.
    */
   public static void multiply(QuaternionReadOnly q1, QuaternionReadOnly q2, QuaternionBasics quaternionToPack)
   {
      multiplyImpl(q1, false, q2, false, quaternionToPack);
   }

   /**
    * Performs the multiplication of {@code q1} conjugated and {@code q2} and stores the result in
    * {@code quaternionToPack}.
    * <p>
    * quaternionToPack = q1* * q2
    * </p>
    * <p>
    * All three arguments can be the same object for in place operations.
    * </p>
    *
    * @param q1               the first quaternion in the multiplication. Not modified.
    * @param q2               the second quaternion in the multiplication. Not modified.
    * @param quaternionToPack the quaternion in which the result is stores. Modified.
    */
   public static void multiplyConjugateLeft(QuaternionReadOnly q1, QuaternionReadOnly q2, QuaternionBasics quaternionToPack)
   {
      multiplyImpl(q1, true, q2, false, quaternionToPack);
   }

   /**
    * Performs the multiplication of {@code q1} and {@code q2} conjugated and stores the result in
    * {@code quaternionToPack}.
    * <p>
    * quaternionToPack = q1 * q2*
    * </p>
    * <p>
    * All three arguments can be the same object for in place operations.
    * </p>
    *
    * @param q1               the first quaternion in the multiplication. Not modified.
    * @param q2               the second quaternion in the multiplication. Not modified.
    * @param quaternionToPack the quaternion in which the result is stores. Modified.
    */
   public static void multiplyConjugateRight(QuaternionReadOnly q1, QuaternionReadOnly q2, QuaternionBasics quaternionToPack)
   {
      multiplyImpl(q1, false, q2, true, quaternionToPack);
   }

   /**
    * Performs the multiplication of the conjugate of {@code q1} and the conjugate of {@code q2} and
    * stores the result in {@code quaternionToPack}.
    * <p>
    * quaternionToPack = q1* * q2*
    * </p>
    * <p>
    * All three arguments can be the same object for in place operations.
    * </p>
    *
    * @param q1               the first quaternion in the multiplication. Not modified.
    * @param q2               the second quaternion in the multiplication. Not modified.
    * @param quaternionToPack the quaternion in which the result is stores. Modified.
    */
   public static void multiplyConjugateBoth(QuaternionReadOnly q1, QuaternionReadOnly q2, QuaternionBasics quaternionToPack)
   {
      multiplyImpl(q1, true, q2, true, quaternionToPack);
   }

   /**
    * Performs the multiplication of {@code orientation1} and {@code orientation2} and stores the
    * result in {@code quaternionToPack}.
    * <p>
    * More precisely, {@code orientation1} and {@code orientation2} are first converted to quaternions,
    * then a quaternion multiplication is performed using the two first arguments as entry.
    * </p>
    * <p>
    * All three arguments can be the same object for in place operations.
    * </p>
    *
    * @param orientation1     the first orientation in the multiplication. Not modified.
    * @param inverse1         whether the first orientation should be inverted in the multiplication.
    * @param orientation2     the second orientation in the multiplication. Not modified.
    * @param inverse2         whether the second orientation should be inverted in the multiplication.
    * @param quaternionToPack the quaternion in which the result is stored. Modified.
    */
   public static void multiply(Orientation3DReadOnly orientation1, boolean inverse1, Orientation3DReadOnly orientation2, boolean inverse2,
                               QuaternionBasics quaternionToPack)
   {
      if (orientation1 instanceof QuaternionReadOnly)
      {
         multiply((QuaternionReadOnly) orientation1, inverse1, orientation2, inverse2, quaternionToPack);
         return;
      }

      double q2s, q2x, q2y, q2z;
      if (orientation2 instanceof QuaternionReadOnly)
      { // In this case orientation2 might be the same object as quaternionToPack, so let's save its components first.
         QuaternionReadOnly q2 = (QuaternionReadOnly) orientation2;
         q2x = q2.getX();
         q2y = q2.getY();
         q2z = q2.getZ();
         q2s = q2.getS();
      }
      else
      {
         quaternionToPack.set(orientation2);
         q2x = quaternionToPack.getX();
         q2y = quaternionToPack.getY();
         q2z = quaternionToPack.getZ();
         q2s = quaternionToPack.getS();
      }

      // Now we can safely use the quaternionToPack argument to convert the orientation1.
      quaternionToPack.set(orientation1);
      double q1x = quaternionToPack.getX();
      double q1y = quaternionToPack.getY();
      double q1z = quaternionToPack.getZ();
      double q1s = quaternionToPack.getS();
      multiplyImpl(q1x, q1y, q1z, q1s, inverse1, q2x, q2y, q2z, q2s, inverse2, quaternionToPack);
   }

   /**
    * Performs the multiplication of {@code orientation1} and {@code orientation2} and stores the
    * result in {@code quaternionToPack}.
    * <p>
    * More precisely, {@code orientation1} is first converted to a quaternion, then a quaternion
    * multiplication is performed using the two first arguments as entry.
    * </p>
    * <p>
    * All three arguments can be the same object for in place operations.
    * </p>
    *
    * @param orientation1     the first orientation in the multiplication. Not modified.
    * @param inverse1         whether the first orientation should be inverted in the multiplication.
    * @param orientation2     the second orientation in the multiplication. Not modified.
    * @param inverse2         whether the second orientation should be inverted in the multiplication.
    * @param quaternionToPack the quaternion in which the result is stored. Modified.
    */
   public static void multiply(Orientation3DReadOnly orientation1, boolean inverse1, QuaternionReadOnly orientation2, boolean inverse2,
                               QuaternionBasics quaternionToPack)
   {
      if (orientation1 instanceof QuaternionReadOnly)
      {
         multiplyImpl((QuaternionReadOnly) orientation1, inverse1, orientation2, inverse2, quaternionToPack);
         return;
      }

      // In this case orientation2 might be the same object as quaternionToPack, so let's save its components first.
      double q2x = orientation2.getX();
      double q2y = orientation2.getY();
      double q2z = orientation2.getZ();
      double q2s = orientation2.getS();
      // Now we can safely use the quaternionToPack argument to convert the orientation1.
      quaternionToPack.set(orientation1);
      double q1x = quaternionToPack.getX();
      double q1y = quaternionToPack.getY();
      double q1z = quaternionToPack.getZ();
      double q1s = quaternionToPack.getS();
      multiplyImpl(q1x, q1y, q1z, q1s, inverse1, q2x, q2y, q2z, q2s, inverse2, quaternionToPack);
   }

   /**
    * Performs the multiplication of {@code orientation1} and {@code orientation2} and stores the
    * result in {@code quaternionToPack}.
    * <p>
    * More precisely, {@code orientation2} is first converted to a quaternion, then a quaternion
    * multiplication is performed using the two first arguments as entry.
    * </p>
    * <p>
    * All three arguments can be the same object for in place operations.
    * </p>
    *
    * @param orientation1     the first orientation in the multiplication. Not modified.
    * @param inverse1         whether the first orientation should be inverted in the multiplication.
    * @param orientation2     the second orientation in the multiplication. Not modified.
    * @param inverse2         whether the second orientation should be inverted in the multiplication.
    * @param quaternionToPack the quaternion in which the result is stored. Modified.
    */
   public static void multiply(QuaternionReadOnly orientation1, boolean inverse1, Orientation3DReadOnly orientation2, boolean inverse2,
                               QuaternionBasics quaternionToPack)
   {
      if (orientation2 instanceof QuaternionReadOnly)
      {
         multiplyImpl(orientation1, inverse1, (QuaternionReadOnly) orientation2, inverse2, quaternionToPack);
         return;
      }

      // In this case orientation1 might be the same object as quaternionToPack, so let's save its components first.
      double q1x = orientation1.getX();
      double q1y = orientation1.getY();
      double q1z = orientation1.getZ();
      double q1s = orientation1.getS();
      // Now we can safely use the quaternionToPack argument to convert the orientation2.
      quaternionToPack.set(orientation2);
      double q2x = quaternionToPack.getX();
      double q2y = quaternionToPack.getY();
      double q2z = quaternionToPack.getZ();
      double q2s = quaternionToPack.getS();
      multiplyImpl(q1x, q1y, q1z, q1s, inverse1, q2x, q2y, q2z, q2s, inverse2, quaternionToPack);
   }

   /**
    * Performs the multiplication of {@code q1} and {@code q2} and stores the result in
    * {@code quaternionToPack}.
    * <p>
    * <b> This method is for internal use only. </b>
    * </p>
    * <p>
    * Provides the option to conjugate either quaternion when multiplying them.
    * <p>
    * All three arguments can be the same object for in place operations.
    * </p>
    *
    * @param q1               the first quaternion in the multiplication. Not modified.
    * @param conjugateQ1      whether to conjugate {@code q1} or not.
    * @param q2               the second quaternion in the multiplication. Not modified.
    * @param conjugateQ2      whether to conjugate {@code q2} or not.
    * @param quaternionToPack the quaternion in which the result is stores. Modified.
    */
   private static void multiplyImpl(QuaternionReadOnly q1, boolean conjugateQ1, QuaternionReadOnly q2, boolean conjugateQ2, QuaternionBasics quaternionToPack)
   {
      multiplyImpl(q1.getX(), q1.getY(), q1.getZ(), q1.getS(), conjugateQ1, q2.getX(), q2.getY(), q2.getZ(), q2.getS(), conjugateQ2, quaternionToPack);
   }

   /**
    * Performs the multiplication of {@code q1} and {@code q2} and stores the result in
    * {@code quaternionToPack}.
    * <p>
    * <b> This method is for internal use only. </b>
    * </p>
    * <p>
    * Provides the option to conjugate either quaternion when multiplying them.
    * </p>
    * <p>
    * All three arguments can be the same object for in place operations.
    * </p>
    *
    * @param q1x               the x-component of the first quaternion in the multiplication. Not
    *                          modified.
    * @param q1y               the y-component of the first quaternion in the multiplication. Not
    *                          modified.
    * @param q1z               the z-component of the first quaternion in the multiplication. Not
    *                          modified.
    * @param q1s               the s-component of the first quaternion in the multiplication. Not
    *                          modified.
    * @param conjugateQ1       whether to conjugate {@code q1} or not.
    * @param q2x               the x-component of the second quaternion in the multiplication. Not
    *                          modified.
    * @param q2y               the y-component of the second quaternion in the multiplication. Not
    *                          modified.
    * @param q2z               the z-component of the second quaternion in the multiplication. Not
    *                          modified.
    * @param q2s               the s-component of the second quaternion in the multiplication. Not
    *                          modified.
    * @param conjugateQ2       whether to conjugate {@code q2} or not.
    * @param orientationToPack the orientation in which the result is stored. Modified.
    */
   static void multiplyImpl(double q1x, double q1y, double q1z, double q1s, boolean conjugateQ1, double q2x, double q2y, double q2z, double q2s,
                            boolean conjugateQ2, Orientation3DBasics orientationToPack)
   {
      if (conjugateQ1)
      {
         q1x = -q1x;
         q1y = -q1y;
         q1z = -q1z;
      }

      if (conjugateQ2)
      {
         q2x = -q2x;
         q2y = -q2y;
         q2z = -q2z;
      }

      double x = q1s * q2x + q1x * q2s + q1y * q2z - q1z * q2y;
      double y = q1s * q2y - q1x * q2z + q1y * q2s + q1z * q2x;
      double z = q1s * q2z + q1x * q2y - q1y * q2x + q1z * q2s;
      double s = q1s * q2s - q1x * q2x - q1y * q2y - q1z * q2z;
      orientationToPack.setQuaternion(x, y, z, s);
   }

   /**
    * Performs the multiplication, in the sense of quaternion multiplication, of {@code t1} and
    * {@code t2} and stores the result in {@code vectorToPack}.
    * <p>
    * vectorToPack = t1 * t2
    * </p>
    * <p>
    * This
    * <p>
    * All three arguments can be the same object for in place operations.
    * </p>
    *
    * @param t1           the first tuple in the multiplication. Not modified.
    * @param t2           the second tuple in the multiplication. Not modified.
    * @param vectorToPack the vector in which the result is stores. Modified.
    */
   public static void multiply(Tuple4DReadOnly t1, Tuple4DReadOnly t2, Vector4DBasics vectorToPack)
   {
      multiplyImpl(t1, false, t2, false, vectorToPack);
   }

   /**
    * Performs the multiplication, in the sense of quaternion multiplication, of {@code t1} conjugated
    * and {@code t2} and stores the result in {@code vectorToPack}.
    * <p>
    * vectorToPack = t1* * t2
    * </p>
    * <p>
    * This
    * <p>
    * All three arguments can be the same object for in place operations.
    * </p>
    *
    * @param t1           the first tuple in the multiplication. Not modified.
    * @param t2           the second tuple in the multiplication. Not modified.
    * @param vectorToPack the vector in which the result is stores. Modified.
    */
   public static void multiplyConjugateLeft(Tuple4DReadOnly t1, Tuple4DReadOnly t2, Vector4DBasics vectorToPack)
   {
      multiplyImpl(t1, true, t2, false, vectorToPack);
   }

   /**
    * Performs the multiplication, in the sense of quaternion multiplication, of {@code t1} and
    * {@code t2} conjugated and stores the result in {@code vectorToPack}.
    * <p>
    * vectorToPack = t1 * t2*
    * </p>
    * <p>
    * This
    * <p>
    * All three arguments can be the same object for in place operations.
    * </p>
    *
    * @param t1           the first tuple in the multiplication. Not modified.
    * @param t2           the second tuple in the multiplication. Not modified.
    * @param vectorToPack the vector in which the result is stores. Modified.
    */
   public static void multiplyConjugateRight(Tuple4DReadOnly t1, Tuple4DReadOnly t2, Vector4DBasics vectorToPack)
   {
      multiplyImpl(t1, false, t2, true, vectorToPack);
   }

   /**
    * Performs the multiplication, in the sense of quaternion multiplication, of {@code t1} and
    * {@code t2} and stores the result in {@code vectorToPack}.
    * <p>
    * <b> This method is for internal use only. </b>
    * </p>
    * <p>
    * Provides the option to conjugate either tuple when multiplying them.
    * <p>
    * All three arguments can be the same object for in place operations.
    * </p>
    *
    * @param t1           the first tuple in the multiplication. Not modified.
    * @param conjugateT1  whether to conjugate {@code t1} or not.
    * @param t2           the second tuple in the multiplication. Not modified.
    * @param conjugateT2  whether to conjugate {@code t2} or not.
    * @param vectorToPack the vector in which the result is stores. Modified.
    */
   private static void multiplyImpl(Tuple4DReadOnly t1, boolean conjugateT1, Tuple4DReadOnly t2, boolean conjugateT2, Vector4DBasics vectorToPack)
   {
      multiplyImpl(t1.getX(), t1.getY(), t1.getZ(), t1.getS(), conjugateT1, t2.getX(), t2.getY(), t2.getZ(), t2.getS(), conjugateT2, vectorToPack);
   }

   /**
    * Performs the multiplication of {@code t1} and {@code t2} and stores the result in
    * {@code vectorToPack}.
    * <p>
    * <b> This method is for internal use only. </b>
    * </p>
    * <p>
    * Provides the option to conjugate either tuple when multiplying them.
    * </p>
    * <p>
    * All three arguments can be the same object for in place operations.
    * </p>
    *
    * @param x1           the x-component of the first tuple in the multiplication. Not modified.
    * @param y1           the y-component of the first tuple in the multiplication. Not modified.
    * @param z1           the z-component of the first tuple in the multiplication. Not modified.
    * @param s1           the s-component of the first tuple in the multiplication. Not modified.
    * @param conjugateT1  whether to conjugate {@code t1} or not.
    * @param x2           the x-component of the second tuple in the multiplication. Not modified.
    * @param y2           the y-component of the second tuple in the multiplication. Not modified.
    * @param z2           the z-component of the second tuple in the multiplication. Not modified.
    * @param s2           the s-component of the second tuple in the multiplication. Not modified.
    * @param conjugateT2  whether to conjugate {@code t2} or not.
    * @param vectorToPack the vector in which the result is stores. Modified.
    */
   private static void multiplyImpl(double x1, double y1, double z1, double s1, boolean conjugateT1, double x2, double y2, double z2, double s2,
                                    boolean conjugateT2, Vector4DBasics vectorToPack)
   {
      if (conjugateT1)
      {
         x1 = -x1;
         y1 = -y1;
         z1 = -z1;
      }

      if (conjugateT2)
      {
         x2 = -x2;
         y2 = -y2;
         z2 = -z2;
      }

      double x = s1 * x2 + x1 * s2 + y1 * z2 - z1 * y2;
      double y = s1 * y2 - x1 * z2 + y1 * s2 + z1 * x2;
      double z = s1 * z2 + x1 * y2 - y1 * x2 + z1 * s2;
      double s = s1 * s2 - x1 * x2 - y1 * y2 - z1 * z2;
      vectorToPack.set(x, y, z, s);
   }

   /**
    * Transforms the tuple {@code tupleOriginal} using {@code quaternion} and stores the result in
    * {@code tupleTransformed}.
    * <p>
    * Both tuples can be the same object for performing in place transformation.
    * </p>
    * <p>
    * tupleTransformed = quaternion * tupleOriginal * quaternion<sup>-1</sup>
    * </p>
    *
    * @param quaternion       the quaternion used to transform the tuple. Not modified.
    * @param tupleOriginal    the tuple to transform. Not modified.
    * @param tupleTransformed the tuple in which the result is stored. Modified.
    */
   public static void transform(QuaternionReadOnly quaternion, Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      transformImpl(quaternion, false, tupleOriginal, tupleTransformed);
   }

   /**
    * Performs the inverse of the transform of the tuple {@code tupleOriginal} using {@code quaternion}
    * and stores the result in {@code tupleTransformed}.
    * <p>
    * This is equivalent to calling
    * {@link #transform(QuaternionReadOnly, Tuple3DReadOnly, Tuple3DBasics)} with the inverse of the
    * given quaternion.
    * </p>
    * <p>
    * Both tuples can be the same object for performing in place transformation.
    * </p>
    * <p>
    * tupleTransformed = quaternion<sup>-1</sup> * tupleOriginal * quaternion
    * </p>
    *
    * @param quaternion       the quaternion used to transform the tuple. Not modified.
    * @param tupleOriginal    the tuple to transform. Not modified.
    * @param tupleTransformed the tuple in which the result is stored. Modified.
    */
   public static void inverseTransform(QuaternionReadOnly quaternion, Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      transformImpl(quaternion, true, tupleOriginal, tupleTransformed);
   }

   /**
    * Transforms the tuple {@code tupleOriginal} using {@code quaternion} and stores the result in
    * {@code tupleTransformed}.
    * <p>
    * <b> This method is for internal use only. </b>
    * </p>
    * <p>
    * Both tuples can be the same object for performing in place transformation.
    * </p>
    * <p>
    * tupleTransformed = quaternion * tupleOriginal * quaternion<sup>-1</sup>
    * </p>
    *
    * @param quaternion          the quaternion used to transform the tuple. Not modified.
    * @param conjugateQuaternion whether to conjugate the quaternion or not.
    * @param tupleOriginal       the tuple to transform. Not modified.
    * @param tupleTransformed    the tuple in which the result is stored. Modified.
    */
   private static void transformImpl(QuaternionReadOnly quaternion, boolean conjugateQuaternion, Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      double qx = quaternion.getX();
      double qy = quaternion.getY();
      double qz = quaternion.getZ();
      double qs = quaternion.getS();

      if (conjugateQuaternion)
      {
         qx = -qx;
         qy = -qy;
         qz = -qz;
      }

      double norm = quaternion.norm();

      if (norm < EPS)
      {
         tupleTransformed.set(tupleOriginal);
         return;
      }

      norm = 1.0 / norm;
      qx *= norm;
      qy *= norm;
      qz *= norm;
      qs *= norm;

      // t = 2.0 * cross(q.xyz, v);
      // v' = v + q.s * t + cross(q.xyz, t);
      double x = tupleOriginal.getX();
      double y = tupleOriginal.getY();
      double z = tupleOriginal.getZ();

      double crossX = 2.0 * (qy * z - qz * y);
      double crossY = 2.0 * (qz * x - qx * z);
      double crossZ = 2.0 * (qx * y - qy * x);

      double crossCrossX = qy * crossZ - qz * crossY;
      double crossCrossY = qz * crossX - qx * crossZ;
      double crossCrossZ = qx * crossY - qy * crossX;

      tupleTransformed.setX(x + qs * crossX + crossCrossX);
      tupleTransformed.setY(y + qs * crossY + crossCrossY);
      tupleTransformed.setZ(z + qs * crossZ + crossCrossZ);
   }

   /**
    * Transforms the tuple {@code tupleOriginal} using {@code quaternion} and adds the result to
    * {@code tupleTransformed}.
    * <p>
    * Both tuples can be the same object for performing in place transformation.
    * </p>
    * <p>
    * tupleTransformed = tupleTransformed + quaternion * tupleOriginal * quaternion<sup>-1</sup>
    * </p>
    *
    * @param quaternion       the quaternion used to transform the tuple. Not modified.
    * @param tupleOriginal    the tuple to transform. Not modified.
    * @param tupleTransformed the tuple in which the result is stored. Modified.
    */
   public static void addTransform(QuaternionReadOnly quaternion, Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      addTransform(false, quaternion, false, tupleOriginal, tupleTransformed);
   }

   /**
    * Transforms the tuple {@code tupleOriginal} using {@code quaternion} and subtracts the result to
    * {@code tupleTransformed}.
    * <p>
    * Both tuples can be the same object for performing in place transformation.
    * </p>
    * <p>
    * tupleTransformed = tupleTransformed - quaternion * tupleOriginal * quaternion<sup>-1</sup>
    * </p>
    *
    * @param quaternion       the quaternion used to transform the tuple. Not modified.
    * @param tupleOriginal    the tuple to transform. Not modified.
    * @param tupleTransformed the tuple in which the result is stored. Modified.
    */
   public static void subTransform(QuaternionReadOnly quaternion, Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      addTransform(true, quaternion, false, tupleOriginal, tupleTransformed);
   }

   /**
    * Transforms the tuple {@code tupleOriginal} using {@code quaternion} and adds the result to
    * {@code tupleTransformed}.
    * <p>
    * <b> This method is for internal use only. </b>
    * </p>
    * <p>
    * Both tuples can be the same object for performing in place transformation.
    * </p>
    * <p>
    * tupleTransformed = tupleTransformed +/- quaternion * tupleOriginal * quaternion<sup>-1</sup>
    * </p>
    *
    * @param subtract            whether to perform a subtraction instead or not.
    * @param quaternion          the quaternion used to transform the tuple. Not modified.
    * @param conjugateQuaternion whether to conjugate the quaternion or not.
    * @param tupleOriginal       the tuple to transform. Not modified.
    * @param tupleTransformed    the tuple in which the result is stored. Modified.
    */
   private static void addTransform(boolean subtract, QuaternionReadOnly quaternion, boolean conjugateQuaternion, Tuple3DReadOnly tupleOriginal,
                                    Tuple3DBasics tupleTransformed)
   {
      double qx = quaternion.getX();
      double qy = quaternion.getY();
      double qz = quaternion.getZ();
      double qs = quaternion.getS();

      if (conjugateQuaternion)
      {
         qx = -qx;
         qy = -qy;
         qz = -qz;
      }

      double norm = quaternion.norm();

      if (norm < EPS)
      {
         return;
      }

      norm = 1.0 / norm;
      qx *= norm;
      qy *= norm;
      qz *= norm;
      qs *= norm;

      // t = 2.0 * cross(q.xyz, v);
      // v' = v + q.s * t + cross(q.xyz, t);
      double x = tupleOriginal.getX();
      double y = tupleOriginal.getY();
      double z = tupleOriginal.getZ();

      double crossX = 2.0 * (qy * z - qz * y);
      double crossY = 2.0 * (qz * x - qx * z);
      double crossZ = 2.0 * (qx * y - qy * x);

      double crossCrossX = qy * crossZ - qz * crossY;
      double crossCrossY = qz * crossX - qx * crossZ;
      double crossCrossZ = qx * crossY - qy * crossX;

      x = x + qs * crossX + crossCrossX;
      y = y + qs * crossY + crossCrossY;
      z = z + qs * crossZ + crossCrossZ;

      if (subtract)
      {
         tupleTransformed.sub(x, y, z);
      }
      else
      {
         tupleTransformed.add(x, y, z);
      }
   }

   /**
    * Transforms the tuple {@code tupleOriginal} using {@code quaternion} and stores the result in
    * {@code tupleTransformed}.
    * <p>
    * Both tuples can be the same object for performing in place transformation.
    * </p>
    * <p>
    * tupleTransformed = quaternion * tupleOriginal * quaternion<sup>-1</sup>
    * </p>
    *
    * @param quaternion                the quaternion used to transform the tuple. Not modified.
    * @param tupleOriginal             the tuple to transform. Not modified.
    * @param tupleTransformed          the tuple in which the result is stored. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that the quaternion represents
    *                                  a transformation in the XY plane.
    * @throws NotAMatrix2DException if {@code checkIfTransformInXYPlane == true} and the quaternion
    *                               does not represent a transformation in the XY plane.
    */
   public static void transform(QuaternionReadOnly quaternion, Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed, boolean checkIfTransformInXYPlane)
   {
      transformImpl(quaternion, false, tupleOriginal, tupleTransformed, checkIfTransformInXYPlane);
   }

   /**
    * Performs the inverse of the transform of the tuple {@code tupleOriginal} using {@code quaternion}
    * and stores the result in {@code tupleTransformed}.
    * <p>
    * This is equivalent to calling
    * {@link #transform(QuaternionReadOnly, Tuple2DReadOnly, Tuple2DBasics, boolean)} with the inverse
    * of the given quaternion.
    * </p>
    * <p>
    * Both tuples can be the same object for performing in place transformation.
    * </p>
    * <p>
    * tupleTransformed = quaternion<sup>-1</sup> * tupleOriginal * quaternion
    * </p>
    *
    * @param quaternion                the quaternion used to transform the tuple. Not modified.
    * @param tupleOriginal             the tuple to transform. Not modified.
    * @param tupleTransformed          the tuple in which the result is stored. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that the quaternion represents
    *                                  a transformation in the XY plane.
    * @throws NotAMatrix2DException if {@code checkIfTransformInXYPlane == true} and the quaternion
    *                               does not represent a transformation in the XY plane.
    */
   public static void inverseTransform(QuaternionReadOnly quaternion, Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed,
                                       boolean checkIfTransformInXYPlane)
   {
      transformImpl(quaternion, true, tupleOriginal, tupleTransformed, checkIfTransformInXYPlane);
   }

   /**
    * Transforms the tuple {@code tupleOriginal} using {@code quaternion} and stores the result in
    * {@code tupleTransformed}.
    * <p>
    * <b> This method is for internal use only. </b>
    * </p>
    * <p>
    * Both tuples can be the same object for performing in place transformation.
    * </p>
    * <p>
    * tupleTransformed = quaternion * tupleOriginal * quaternion<sup>-1</sup>
    * </p>
    *
    * @param quaternion                the quaternion used to transform the tuple. Not modified.
    * @param conjugateQuaternion       whether to conjugate the quaternion or not.
    * @param tupleOriginal             the tuple to transform. Not modified.
    * @param tupleTransformed          the tuple in which the result is stored. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that the quaternion represents
    *                                  a transformation in the XY plane.
    * @throws NotAMatrix2DException if {@code checkIfTransformInXYPlane == true} and the quaternion
    *                               does not represent a transformation in the XY plane.
    */
   private static void transformImpl(QuaternionReadOnly quaternion, boolean conjugateQuaternion, Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed,
                                     boolean checkIfTransformInXYPlane)
   {
      if (checkIfTransformInXYPlane)
         quaternion.checkIfOrientation2D(EPS);

      double norm = quaternion.norm();

      if (norm < EPS)
      {
         tupleTransformed.set(tupleOriginal);
         return;
      }

      double qz = quaternion.getZ();
      double qs = quaternion.getS();

      if (conjugateQuaternion)
      {
         qz = -qz;
      }

      norm = 1.0 / norm;
      qz *= norm;
      qs *= norm;

      // t = 2.0 * cross(q.xyz, v);
      // v' = v + q.s * t + cross(q.xyz, t);
      double x = tupleOriginal.getX();
      double y = tupleOriginal.getY();

      double crossX = -2.0 * qz * y;
      double crossY = 2.0 * qz * x;

      double crossCrossX = -qz * crossY;
      double crossCrossY = qz * crossX;

      tupleTransformed.setX(x + qs * crossX + crossCrossX);
      tupleTransformed.setY(y + qs * crossY + crossCrossY);
   }

   /**
    * Transforms the quaternion {@code quaternionOriginal} using {@code quaternion} and stores the
    * result in {@code quaternionTransformed}.
    * <p>
    * Both {@code quaternionOriginal} and {@code quaternionTransformed} can be the same object for
    * performing in place transformation.
    * </p>
    * <p>
    * quaternionTransformed = quaternion * quaternionOriginal
    * </p>
    * <p>
    * Note that this transformation is equivalent to concatenating the orientations of
    * {@code quaternion} and {@code quaternionOriginal}.
    * </p>
    *
    * @param quaternion            the quaternion used to transform the quaternion. Not modified.
    * @param quaternionOriginal    the quaternion to transform. Not modified.
    * @param quaternionTransformed the quaternion in which the result is stored. Modified.
    */
   public static void transform(QuaternionReadOnly quaternion, QuaternionReadOnly quaternionOriginal, QuaternionBasics quaternionTransformed)
   {
      multiplyImpl(quaternion, false, quaternionOriginal, false, quaternionTransformed);
   }

   /**
    * Performs the inverse of the transform of the quaternion {@code quaternionOriginal} using
    * {@code quaternion} and stores the result in {@code quaternionTransformed}.
    * <p>
    * This is equivalent to calling
    * {@link #transform(QuaternionReadOnly, QuaternionReadOnly, QuaternionBasics)} with the inverse of
    * the given quaternion.
    * </p>
    * <p>
    * Both {@code quaternionOriginal} and {@code quaternionTransformed} can be the same object for
    * performing in place transformation.
    * </p>
    * <p>
    * quaternionTransformed = quaternion<sup>-1</sup> * quaternionOriginal
    * </p>
    *
    * @param quaternion            the quaternion used to transform the quaternion. Not modified.
    * @param quaternionOriginal    the quaternion to transform. Not modified.
    * @param quaternionTransformed the quaternion in which the result is stored. Modified.
    */
   public static void inverseTransform(QuaternionReadOnly quaternion, QuaternionReadOnly quaternionOriginal, QuaternionBasics quaternionTransformed)
   {
      multiplyImpl(quaternion, true, quaternionOriginal, false, quaternionTransformed);
   }

   /**
    * Transforms the vector part of {@code vectorOriginal} using {@code quaternion} and stores the
    * result in {@code vectorTransformed}.
    * <p>
    * Both vectors can be the same object for performing in place transformation.
    * </p>
    * <p>
    * vectorTransformed.s = vecorOriginal.s <br>
    * vectorTransformed.xyz = quaternion * vectorOriginal.xyz * quaternion<sup>-1</sup>
    * </p>
    *
    * @param quaternion        the quaternion used to transform the vector. Not modified.
    * @param vectorOriginal    the vector to transform. Not modified.
    * @param vectorTransformed the vector in which the result is stored. Modified.
    */
   public static void transform(QuaternionReadOnly quaternion, Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
   {
      transformImpl(quaternion, false, vectorOriginal, vectorTransformed);
   }

   /**
    * Performs the inverse of the transform of the vector part of {@code vectorOriginal} using
    * {@code quaternion} and stores the result in {@code vectorTransformed}.
    * <p>
    * This is equivalent to calling
    * {@link #transform(QuaternionReadOnly, Tuple3DReadOnly, Tuple3DBasics)} with the inverse of the
    * given quaternion.
    * </p>
    * <p>
    * Both vectors can be the same object for performing in place transformation.
    * </p>
    * <p>
    * vectorTransformed.s = vecorOriginal.s <br>
    * vectorTransformed.xyz = quaternion<sup>-1</sup> * vectorOriginal.xyz * quaternion
    * </p>
    *
    * @param quaternion        the quaternion used to transform the vector. Not modified.
    * @param vectorOriginal    the vector to transform. Not modified.
    * @param vectorTransformed the vector in which the result is stored. Modified.
    */
   public static void inverseTransform(QuaternionReadOnly quaternion, Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
   {
      transformImpl(quaternion, true, vectorOriginal, vectorTransformed);
   }

   /**
    * Transforms the vector part of {@code vectorOriginal} using {@code quaternion} and stores the
    * result in {@code vectorTransformed}.
    * <p>
    * <b> This method is for internal use only. </b>
    * </p>
    * <p>
    * Both vectors can be the same object for performing in place transformation.
    * </p>
    * <p>
    * vectorTransformed.s = vecorOriginal.s <br>
    * vectorTransformed.xyz = quaternion * vectorOriginal.xyz * quaternion<sup>-1</sup>
    * </p>
    *
    * @param quaternion          the quaternion used to transform the vector. Not modified.
    * @param conjugateQuaternion whether to conjugate the quaternion or not.
    * @param vectorOriginal      the vector to transform. Not modified.
    * @param vectorTransformed   the vector in which the result is stored. Modified.
    */
   private static void transformImpl(QuaternionReadOnly quaternion, boolean conjugateQuaternion, Vector4DReadOnly vectorOriginal,
                                     Vector4DBasics vectorTransformed)
   {
      double norm = quaternion.norm();

      if (norm < EPS)
      {
         vectorTransformed.set(vectorOriginal);
         return;
      }

      double qx = quaternion.getX();
      double qy = quaternion.getY();
      double qz = quaternion.getZ();
      double qs = quaternion.getS();

      if (conjugateQuaternion)
      {
         qx = -qx;
         qy = -qy;
         qz = -qz;
      }

      // It's the same as transforming a vector 3D. The scalar of the transformed vector 4D is the same as the original.
      norm = 1.0 / norm;
      qx *= norm;
      qy *= norm;
      qz *= norm;
      qs *= norm;

      // t = 2.0 * cross(q.xyz, v);
      // v' = v + q.s * t + cross(q.xyz, t);
      double x = vectorOriginal.getX();
      double y = vectorOriginal.getY();
      double z = vectorOriginal.getZ();

      double crossX = 2.0 * (qy * z - qz * y);
      double crossY = 2.0 * (qz * x - qx * z);
      double crossZ = 2.0 * (qx * y - qy * x);

      double crossCrossX = qy * crossZ - qz * crossY;
      double crossCrossY = qz * crossX - qx * crossZ;
      double crossCrossZ = qx * crossY - qy * crossX;

      vectorTransformed.setX(x + qs * crossX + crossCrossX);
      vectorTransformed.setY(y + qs * crossY + crossCrossY);
      vectorTransformed.setZ(z + qs * crossZ + crossCrossZ);
      vectorTransformed.setS(vectorOriginal.getS());
   }

   /**
    * Transforms the matrix {@code matrixOriginal} using {@code quaternion} and stores the result in
    * {@code matrixTransformed}.
    * <p>
    * Both matrices can be the same object for performing in place transformation.
    * </p>
    * <p>
    * matrixTransformed = R(quaternion) * matrixOriginal * R(quaternion)<sup>-1</sup> <br>
    * where R(quaternion) is the function to convert a quaternion into a 3-by-3 rotation matrix.
    * </p>
    *
    * @param quaternion        the quaternion used to transform the matrix. Not modified.
    * @param matrixOriginal    the matrix to transform. Not modified.
    * @param matrixTransformed the matrix in which the result is stored. Modified.
    */
   public static void transform(QuaternionReadOnly quaternion, Matrix3DReadOnly matrixOriginal, Matrix3DBasics matrixTransformed)
   {
      transformImpl(quaternion, false, matrixOriginal, matrixTransformed);
   }

   /**
    * Performs the inverse of the transform of the matrix {@code matrixOriginal} using
    * {@code quaternion} and stores the result in {@code matrixTransformed}.
    * <p>
    * This is equivalent to calling
    * {@link #transform(QuaternionReadOnly, Matrix3DReadOnly, Matrix3DBasics)} with the inverse of the
    * given quaternion.
    * </p>
    * <p>
    * Both matrices can be the same object for performing in place transformation.
    * </p>
    * <p>
    * matrixTransformed = R(quaternion)<sup>-1</sup> * matrixOriginal * R(quaternion) <br>
    * where R(quaternion) is the function to convert a quaternion into a 3-by-3 rotation matrix.
    * </p>
    *
    * @param quaternion        the quaternion used to transform the matrix. Not modified.
    * @param matrixOriginal    the matrix to transform. Not modified.
    * @param matrixTransformed the matrix in which the result is stored. Modified.
    */
   public static void inverseTransform(QuaternionReadOnly quaternion, Matrix3DReadOnly matrixOriginal, Matrix3DBasics matrixTransformed)
   {
      transformImpl(quaternion, true, matrixOriginal, matrixTransformed);
   }

   /**
    * Transforms the matrix {@code matrixOriginal} using {@code quaternion} and stores the result in
    * {@code matrixTransformed}.
    * <p>
    * <b> This method is for internal use only. </b>
    * </p>
    * <p>
    * <p>
    * Both matrices can be the same object for performing in place transformation.
    * </p>
    * <p>
    * matrixTransformed = R(quaternion) * matrixOriginal * R(quaternion)<sup>-1</sup> <br>
    * where R(quaternion) is the function to convert a quaternion into a 3-by-3 rotation matrix.
    * </p>
    *
    * @param quaternion          the quaternion used to transform the matrix. Not modified.
    * @param conjugateQuaternion whether to conjugate the quaternion or not.
    * @param matrixOriginal      the matrix to transform. Not modified.
    * @param matrixTransformed   the matrix in which the result is stored. Modified.
    */
   private static void transformImpl(QuaternionReadOnly quaternion, boolean conjugateQuaternion, Matrix3DReadOnly matrixOriginal,
                                     Matrix3DBasics matrixTransformed)
   {
      double qx = quaternion.getX();
      double qy = quaternion.getY();
      double qz = quaternion.getZ();
      double qs = quaternion.getS();
      transformImpl(qx, qy, qz, qs, conjugateQuaternion, matrixOriginal, matrixTransformed);
   }

   /**
    * Transforms the matrix {@code matrixOriginal} using {@code quaternion} and stores the result in
    * {@code matrixTransformed}.
    * <p>
    * <b> This method is for internal use only. </b>
    * </p>
    * <p>
    * <p>
    * Both matrices can be the same object for performing in place transformation.
    * </p>
    * <p>
    * matrixTransformed = R(quaternion) * matrixOriginal * R(quaternion)<sup>-1</sup> <br>
    * where R(quaternion) is the function to convert a quaternion into a 3-by-3 rotation matrix.
    * </p>
    *
    * @param qx                  the x-component of the quaternion used to transform the matrix.
    * @param qy                  the y-component of the quaternion used to transform the matrix.
    * @param qz                  the z-component of the quaternion used to transform the matrix.
    * @param qs                  the s-component of the quaternion used to transform the matrix.
    * @param conjugateQuaternion whether to conjugate the quaternion or not.
    * @param matrixOriginal      the matrix to transform. Not modified.
    * @param matrixTransformed   the matrix in which the result is stored. Modified.
    */
   static void transformImpl(double qx, double qy, double qz, double qs, boolean conjugateQuaternion, Matrix3DReadOnly matrixOriginal,
                             Matrix3DBasics matrixTransformed)
   {
      double norm = EuclidCoreTools.fastNorm(qx, qy, qz, qs);

      if (norm < EPS)
      {
         matrixTransformed.set(matrixOriginal);
         return;
      }

      if (conjugateQuaternion)
      {
         qx = -qx;
         qy = -qy;
         qz = -qz;
      }

      norm = 1.0 / norm;
      qx *= norm;
      qy *= norm;
      qz *= norm;
      qs *= norm;

      double yy2 = 2.0 * qy * qy;
      double zz2 = 2.0 * qz * qz;
      double xx2 = 2.0 * qx * qx;
      double xy2 = 2.0 * qx * qy;
      double sz2 = 2.0 * qs * qz;
      double xz2 = 2.0 * qx * qz;
      double sy2 = 2.0 * qs * qy;
      double yz2 = 2.0 * qy * qz;
      double sx2 = 2.0 * qs * qx;

      double q00 = 1.0 - yy2 - zz2;
      double q01 = xy2 - sz2;
      double q02 = xz2 + sy2;
      double q10 = xy2 + sz2;
      double q11 = 1.0 - xx2 - zz2;
      double q12 = yz2 - sx2;
      double q20 = xz2 - sy2;
      double q21 = yz2 + sx2;
      double q22 = 1.0 - xx2 - yy2;

      double qM00 = q00 * matrixOriginal.getM00() + q01 * matrixOriginal.getM10() + q02 * matrixOriginal.getM20();
      double qM01 = q00 * matrixOriginal.getM01() + q01 * matrixOriginal.getM11() + q02 * matrixOriginal.getM21();
      double qM02 = q00 * matrixOriginal.getM02() + q01 * matrixOriginal.getM12() + q02 * matrixOriginal.getM22();
      double qM10 = q10 * matrixOriginal.getM00() + q11 * matrixOriginal.getM10() + q12 * matrixOriginal.getM20();
      double qM11 = q10 * matrixOriginal.getM01() + q11 * matrixOriginal.getM11() + q12 * matrixOriginal.getM21();
      double qM12 = q10 * matrixOriginal.getM02() + q11 * matrixOriginal.getM12() + q12 * matrixOriginal.getM22();
      double qM20 = q20 * matrixOriginal.getM00() + q21 * matrixOriginal.getM10() + q22 * matrixOriginal.getM20();
      double qM21 = q20 * matrixOriginal.getM01() + q21 * matrixOriginal.getM11() + q22 * matrixOriginal.getM21();
      double qM22 = q20 * matrixOriginal.getM02() + q21 * matrixOriginal.getM12() + q22 * matrixOriginal.getM22();

      double qMqt00 = qM00 * q00 + qM01 * q01 + qM02 * q02;
      double qMqt01 = qM00 * q10 + qM01 * q11 + qM02 * q12;
      double qMqt02 = qM00 * q20 + qM01 * q21 + qM02 * q22;
      double qMqt10 = qM10 * q00 + qM11 * q01 + qM12 * q02;
      double qMqt11 = qM10 * q10 + qM11 * q11 + qM12 * q12;
      double qMqt12 = qM10 * q20 + qM11 * q21 + qM12 * q22;
      double qMqt20 = qM20 * q00 + qM21 * q01 + qM22 * q02;
      double qMqt21 = qM20 * q10 + qM21 * q11 + qM22 * q12;
      double qMqt22 = qM20 * q20 + qM21 * q21 + qM22 * q22;

      matrixTransformed.set(qMqt00, qMqt01, qMqt02, qMqt10, qMqt11, qMqt12, qMqt20, qMqt21, qMqt22);
   }

   /**
    * Transforms the rotation matrix {@code rotationMatrixOriginal} using {@code quaternion} and stores
    * the result in {@code rotationMatrixTransformed}.
    * <p>
    * Both rotation matrices can be the same object for performing in place transformation.
    * </p>
    * <p>
    * rotationMatrixTransformed = R(quaternion) * rotationMatrixOriginal <br>
    * where R(quaternion) is the function to convert a quaternion into a 3-by-3 rotation matrix.
    * </p>
    * <p>
    * Note that this transformation is equivalent to concatenating the orientations of
    * {@code quaternion} and {@code rotationMatrixOriginal}.
    * </p>
    *
    * @param quaternion                the quaternion used to transform the rotation matrix. Not
    *                                  modified.
    * @param rotationMatrixOriginal    the rotation matrix to transform. Not modified.
    * @param rotationMatrixTransformed the rotation matrix in which the result is stored. Modified.
    */
   public static void transform(QuaternionReadOnly quaternion, RotationMatrixReadOnly rotationMatrixOriginal, RotationMatrix rotationMatrixTransformed)
   {
      RotationMatrixTools.multiply(quaternion, false, rotationMatrixOriginal, false, rotationMatrixTransformed);
   }

   /**
    * Performs the inverse of the transform of the rotation matrix {@code rotationMatrixOriginal} using
    * {@code quaternion} and stores the result in {@code rotationMatrixTransformed}.
    * <p>
    * This is equivalent to calling
    * {@link #transform(QuaternionReadOnly, RotationMatrixReadOnly, RotationMatrix)} with the inverse
    * of the given quaternion.
    * </p>
    * <p>
    * Both rotation matrices can be the same object for performing in place transformation.
    * </p>
    * <p>
    * rotationMatrixTransformed = R(quaternion)<sup>-1</sup> * rotationMatrixOriginal <br>
    * where R(quaternion) is the function to convert a quaternion into a 3-by-3 rotation matrix.
    * </p>
    * <p>
    * Note that this transformation is equivalent to concatenating the orientations of
    * {@code quaternion} and {@code rotationMatrixOriginal}.
    * </p>
    *
    * @param quaternion                the quaternion used to transform the rotation matrix. Not
    *                                  modified.
    * @param rotationMatrixOriginal    the rotation matrix to transform. Not modified.
    * @param rotationMatrixTransformed the rotation matrix in which the result is stored. Modified.
    */
   public static void inverseTransform(QuaternionReadOnly quaternion, RotationMatrixReadOnly rotationMatrixOriginal, RotationMatrix rotationMatrixTransformed)
   {
      RotationMatrixTools.multiply(quaternion, true, rotationMatrixOriginal, false, rotationMatrixTransformed);
   }

   /**
    * Prepend a rotation about the z-axis to {@code quaternionOriginal} and stores the result in
    * {@code quaternionToPack}.
    * <p>
    * All the quaternions can be the same object.
    * </p>
    *
    * <pre>
    *                    / qx =     0      \
    * quaternionToPack = | qy =     0      | * quaternionOriginal
    *                    | qz = sin(yaw/2) |
    *                    \ qs = cos(yaw/2) /
    * </pre>
    *
    * @param yaw                the angle to rotate about the z-axis.
    * @param quaternionOriginal the quaternion on which the yaw rotation is prepended. Not modified.
    * @param quaternionToPack   the quaternion in which the result is stored. Modified.
    */
   public static void prependYawRotation(double yaw, QuaternionReadOnly quaternionOriginal, QuaternionBasics quaternionToPack)
   {
      double qx = quaternionOriginal.getX();
      double qy = quaternionOriginal.getY();
      double qz = quaternionOriginal.getZ();
      double qs = quaternionOriginal.getS();

      double halfYaw = 0.5 * yaw;
      double cYaw = Math.cos(halfYaw);
      double sYaw = Math.sin(halfYaw);

      double x = cYaw * qx - sYaw * qy;
      double y = cYaw * qy + sYaw * qx;
      double z = cYaw * qz + sYaw * qs;
      double s = cYaw * qs - sYaw * qz;
      quaternionToPack.setUnsafe(x, y, z, s);
   }

   /**
    * Append a rotation about the z-axis to {@code quaternionOriginal} and stores the result in
    * {@code quaternionToPack}.
    * <p>
    * All the quaternions can be the same object.
    * </p>
    *
    * <pre>
    *                                         / qx =     0      \
    * quaternionToPack = quaternionOriginal * | qy =     0      |
    *                                         | qz = sin(yaw/2) |
    *                                         \ qs = cos(yaw/2) /
    * </pre>
    *
    * @param quaternionOriginal the quaternion on which the yaw rotation is appended. Not modified.
    * @param yaw                the angle to rotate about the z-axis.
    * @param quaternionToPack   the quaternion in which the result is stored. Modified.
    */
   public static void appendYawRotation(QuaternionReadOnly quaternionOriginal, double yaw, QuaternionBasics quaternionToPack)
   {
      double qx = quaternionOriginal.getX();
      double qy = quaternionOriginal.getY();
      double qz = quaternionOriginal.getZ();
      double qs = quaternionOriginal.getS();

      double halfYaw = 0.5 * yaw;
      double cYaw = Math.cos(halfYaw);
      double sYaw = Math.sin(halfYaw);

      double x = qx * cYaw + qy * sYaw;
      double y = -qx * sYaw + qy * cYaw;
      double z = qs * sYaw + qz * cYaw;
      double s = qs * cYaw - qz * sYaw;
      quaternionToPack.setUnsafe(x, y, z, s);
   }

   /**
    * Prepend a rotation about the y-axis to {@code quaternionOriginal} and stores the result in
    * {@code quaternionToPack}.
    * <p>
    * All the quaternions can be the same object.
    * </p>
    *
    * <pre>
    *                    / qx =      0       \
    * quaternionToPack = | qy = sin(pitch/2) | * quaternionOriginal
    *                    | qz =      0       |
    *                    \ qs = cos(pitch/2) /
    * </pre>
    *
    * @param pitch              the angle to rotate about the y-axis.
    * @param quaternionOriginal the quaternion on which the pitch rotation is prepended. Not modified.
    * @param quaternionToPack   the quaternion in which the result is stored. Modified.
    */
   public static void prependPitchRotation(double pitch, QuaternionReadOnly quaternionOriginal, QuaternionBasics quaternionToPack)
   {
      double qx = quaternionOriginal.getX();
      double qy = quaternionOriginal.getY();
      double qz = quaternionOriginal.getZ();
      double qs = quaternionOriginal.getS();

      double halfPitch = 0.5 * pitch;
      double cPitch = Math.cos(halfPitch);
      double sPitch = Math.sin(halfPitch);

      double x = cPitch * qx + sPitch * qz;
      double y = cPitch * qy + sPitch * qs;
      double z = cPitch * qz - sPitch * qx;
      double s = cPitch * qs - sPitch * qy;
      quaternionToPack.setUnsafe(x, y, z, s);
   }

   /**
    * Append a rotation about the y-axis to {@code quaternionOriginal} and stores the result in
    * {@code quaternionToPack}.
    * <p>
    * All the quaternions can be the same object.
    * </p>
    *
    * <pre>
    *                                         / qx =      0       \
    * quaternionToPack = quaternionOriginal * | qy = sin(pitch/2) |
    *                                         | qz =      0       |
    *                                         \ qs = cos(pitch/2) /
    * </pre>
    *
    * @param quaternionOriginal the quaternion on which the pitch rotation is appended. Not modified.
    * @param pitch              the angle to rotate about the y-axis.
    * @param quaternionToPack   the quaternion in which the result is stored. Modified.
    */
   public static void appendPitchRotation(QuaternionReadOnly quaternionOriginal, double pitch, QuaternionBasics quaternionToPack)
   {
      double qx = quaternionOriginal.getX();
      double qy = quaternionOriginal.getY();
      double qz = quaternionOriginal.getZ();
      double qs = quaternionOriginal.getS();

      double halfPitch = 0.5 * pitch;
      double cPitch = Math.cos(halfPitch);
      double sPitch = Math.sin(halfPitch);

      double x = qx * cPitch - qz * sPitch;
      double y = qs * sPitch + qy * cPitch;
      double z = qx * sPitch + qz * cPitch;
      double s = qs * cPitch - qy * sPitch;
      quaternionToPack.setUnsafe(x, y, z, s);
   }

   /**
    * Append a rotation about the x-axis to {@code quaternionOriginal} and stores the result in
    * {@code quaternionToPack}.
    * <p>
    * All the quaternions can be the same object.
    * </p>
    *
    * <pre>
    *                    / qx = sin(roll/2) \
    * quaternionToPack = | qy =      0      | * quaternionOriginal
    *                    | qz =      0      |
    *                    \ qs = cos(roll/2) /
    * </pre>
    *
    * @param roll               the angle to rotate about the x-axis.
    * @param quaternionOriginal the quaternion on which the roll rotation is prepended. Not modified.
    * @param quaternionToPack   the quaternion in which the result is stored. Modified.
    */
   public static void prependRollRotation(double roll, QuaternionReadOnly quaternionOriginal, QuaternionBasics quaternionToPack)
   {
      double qx = quaternionOriginal.getX();
      double qy = quaternionOriginal.getY();
      double qz = quaternionOriginal.getZ();
      double qs = quaternionOriginal.getS();

      double halfRoll = 0.5 * roll;
      double cRoll = Math.cos(halfRoll);
      double sRoll = Math.sin(halfRoll);

      double x = cRoll * qx + sRoll * qs;
      double y = cRoll * qy - sRoll * qz;
      double z = cRoll * qz + sRoll * qy;
      double s = cRoll * qs - sRoll * qx;
      quaternionToPack.setUnsafe(x, y, z, s);
   }

   /**
    * Append a rotation about the x-axis to {@code quaternionOriginal} and stores the result in
    * {@code quaternionToPack}.
    * <p>
    * All the quaternions can be the same object.
    * </p>
    *
    * <pre>
    *                                         / qx = sin(roll/2) \
    * quaternionToPack = quaternionOriginal * | qy =      0      |
    *                                         | qz =      0      |
    *                                         \ qs = cos(roll/2) /
    * </pre>
    *
    * @param quaternionOriginal the quaternion on which the roll rotation is appended. Not modified.
    * @param roll               the angle to rotate about the x-axis.
    * @param quaternionToPack   the quaternion in which the result is stored. Modified.
    */
   public static void appendRollRotation(QuaternionReadOnly quaternionOriginal, double roll, QuaternionBasics quaternionToPack)
   {
      double qx = quaternionOriginal.getX();
      double qy = quaternionOriginal.getY();
      double qz = quaternionOriginal.getZ();
      double qs = quaternionOriginal.getS();

      double halfRoll = 0.5 * roll;
      double cRoll = Math.cos(halfRoll);
      double sRoll = Math.sin(halfRoll);

      double x = qs * sRoll + qx * cRoll;
      double y = qy * cRoll + qz * sRoll;
      double z = -qy * sRoll + qz * cRoll;
      double s = qs * cRoll - qx * sRoll;
      quaternionToPack.setUnsafe(x, y, z, s);
   }

   /**
    * Computes the distance between the two given quaternions.
    *
    * @param q1 the quaternion to be used in the comparison. Not modified.
    * @param q2 the quaternion to be used in the comparison. Not modified.
    * @return the angle representing the distance between the two quaternions. It is contained in [0,
    *         2<i>pi</i>]
    */
   public static double distancePrecise(QuaternionReadOnly q1, QuaternionReadOnly q2)
   {
      double x = q1.getS() * q2.getX() - q1.getX() * q2.getS() - q1.getY() * q2.getZ() + q1.getZ() * q2.getY();
      double y = q1.getS() * q2.getY() + q1.getX() * q2.getZ() - q1.getY() * q2.getS() - q1.getZ() * q2.getX();
      double z = q1.getS() * q2.getZ() - q1.getX() * q2.getY() + q1.getY() * q2.getX() - q1.getZ() * q2.getS();
      double s = q1.getS() * q2.getS() + q1.getX() * q2.getX() + q1.getY() * q2.getY() + q1.getZ() * q2.getZ();

      double sinHalfTheta = EuclidCoreTools.norm(x, y, z);
      return 2.0 * Math.atan2(sinHalfTheta, s);
   }
}
