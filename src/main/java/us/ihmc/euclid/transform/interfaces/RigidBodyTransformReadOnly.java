package us.ihmc.euclid.transform.interfaces;

import us.ihmc.euclid.exceptions.NotAnOrientation2DException;
import us.ihmc.euclid.matrix.interfaces.Matrix3DBasics;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixBasics;
import us.ihmc.euclid.orientation.interfaces.Orientation3DBasics;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.Matrix3DFeatures;
import us.ihmc.euclid.tools.TupleTools;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DBasics;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DReadOnly;

/**
 * Read-only interface for a rigid-body transform.
 * <p>
 * A rigid-body transform represents a transform that can rotate and/or translate geometries.
 * </p>
 * <p>
 * The data structure used to represents this transform is not enforced here, such that the rotation
 * part can be any implementation of orientation 3D.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface RigidBodyTransformReadOnly extends Transform
{
   /**
    * The tolerance used to determine if the translation part is negligible.
    */
   static final double EPS_ZERO_TRANSLATION = 1.0e-10;

   /**
    * Gets the read-only reference to the rotation part of this transform.
    *
    * @return the rotation part of this transform.
    */
   Orientation3DReadOnly getRotation();

   /**
    * Gets the read-only reference of the translation part of this rigid-body transform.
    *
    * @return the translation part of this transform.
    */
   Tuple3DReadOnly getTranslation();

   /**
    * Requests whether this transform has a non-zero rotation or not.
    *
    * @return {@code true} if the rotation part is not zero, {@code false} if the rotation part is zero
    *         and can be ignored when transforming an object.
    */
   default boolean hasRotation()
   {
      return !getRotation().isZeroOrientation();
   }

   /**
    * Requests whether this transform has a non-zero translation or not.
    *
    * @return {@code true} if the translation part is not zero, {@code false} if the translation part
    *         is zero and can be ignore when transforming an object.
    */
   default boolean hasTranslation()
   {
      return !TupleTools.isTupleZero(getTranslation(), EPS_ZERO_TRANSLATION);
   }

   /**
    * Tests if at least one element of this transform is equal to {@linkplain Double#NaN}.
    *
    * @return {@code true} if at least one element of this transform is equal to
    *         {@linkplain Double#NaN}, {@code false} otherwise.
    */
   default boolean containsNaN()
   {
      return getRotation().containsNaN() || getTranslation().containsNaN();
   }

   /**
    * Tests if the rotation part of this transform describes a transformation in the XY plane.
    * <p>
    * The rotation part is considered to be a 2D transformation in the XY plane if:
    * <ul>
    * <li>the last diagonal coefficient m22 is equal to 1.0 +/- {@link Matrix3DFeatures#EPS_CHECK_2D},
    * <li>the coefficients {@code m20}, {@code m02}, {@code m21}, and {@code m12} are equal to 0.0 +/-
    * {@link Matrix3DFeatures#EPS_CHECK_2D}.
    * </ul>
    * </p>
    *
    * @return {@code true} if the rotation part describes a 2D transformation in the XY plane,
    *         {@code false} otherwise.
    */
   default boolean isRotation2D()
   {
      return !hasRotation() || getRotation().isOrientation2D();
   }

   /**
    * Asserts that the rotation part of this transform describes a transformation in the XY plane.
    *
    * @throws NotAnOrientation2DException if the rotation part represents a 3D transformation.
    */
   default void checkIfRotation2D()
   {
      if (hasRotation())
         getRotation().checkIfOrientation2D();
   }

   /** {@inheritDoc} */
   @Override
   default void transform(Point3DReadOnly pointOriginal, Point3DBasics pointTransformed)
   {
      if (hasRotation())
         getRotation().transform(pointOriginal, pointTransformed);
      else
         pointTransformed.set(pointOriginal);

      if (hasTranslation())
         pointTransformed.add(getTranslation());
   }

   /** {@inheritDoc} */
   @Override
   default void transform(Vector3DReadOnly vectorOriginal, Vector3DBasics vectorTransformed)
   {
      if (hasRotation())
         getRotation().transform(vectorOriginal, vectorTransformed);
      else
         vectorTransformed.set(vectorOriginal);
   }

   /** {@inheritDoc} */
   @Override
   default void transform(Orientation3DReadOnly orientationOriginal, Orientation3DBasics orientationTransformed)
   {
      if (hasRotation())
         getRotation().transform(orientationOriginal, orientationTransformed);
      else
         orientationTransformed.set(orientationOriginal);
   }

   /** {@inheritDoc} */
   @Override
   default void transform(Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
   {
      if (hasRotation())
         getRotation().transform(vectorOriginal, vectorTransformed);
      else
         vectorTransformed.set(vectorOriginal);

      if (hasTranslation())
      {
         vectorTransformed.addX(vectorTransformed.getS() * getTranslation().getX());
         vectorTransformed.addY(vectorTransformed.getS() * getTranslation().getY());
         vectorTransformed.addZ(vectorTransformed.getS() * getTranslation().getZ());
      }
   }

   /** {@inheritDoc} */
   @Override
   default void transform(Point2DReadOnly point2DOriginal, Point2DBasics point2DTransformed, boolean checkIfTransformInXYPlane)
   {
      if (hasRotation())
         getRotation().transform(point2DOriginal, point2DTransformed, checkIfTransformInXYPlane);
      else
         point2DTransformed.set(point2DOriginal);
      if (hasTranslation())
         point2DTransformed.add(getTranslation().getX(), getTranslation().getY());
   }

   /** {@inheritDoc} */
   @Override
   default void transform(Vector2DReadOnly vector2DOriginal, Vector2DBasics vector2DTransformed, boolean checkIfTransformInXYPlane)
   {
      if (hasRotation())
         getRotation().transform(vector2DOriginal, vector2DTransformed, checkIfTransformInXYPlane);
      else
         vector2DTransformed.set(vector2DOriginal);
   }

   /** {@inheritDoc} */
   @Override
   default void transform(Matrix3DReadOnly matrixOriginal, Matrix3DBasics matrixTransformed)
   {
      if (hasRotation())
         getRotation().transform(matrixOriginal, matrixTransformed);
      else
         matrixTransformed.set(matrixOriginal);
   }

   /** {@inheritDoc} */
   @Override
   default void transform(RigidBodyTransformReadOnly original, RigidBodyTransformBasics transformed)
   {
      transformed.set(original);
      transformed.preMultiply(this);
   }

   /** {@inheritDoc} */
   @Override
   default void transform(AffineTransformReadOnly original, AffineTransformBasics transformed)
   {
      transformed.set(original);
      transformed.preMultiply(this);
   }

   /** {@inheritDoc} */
   @Override
   default void inverseTransform(Point3DReadOnly pointOriginal, Point3DBasics pointTransformed)
   {
      pointTransformed.set(pointOriginal);
      if (hasTranslation())
         pointTransformed.sub(getTranslation());
      if (hasRotation())
         getRotation().inverseTransform(pointTransformed);
   }

   /** {@inheritDoc} */
   @Override
   default void inverseTransform(Vector3DReadOnly vectorOriginal, Vector3DBasics vectorTransformed)
   {
      if (hasRotation())
         getRotation().inverseTransform(vectorOriginal, vectorTransformed);
      else
         vectorTransformed.set(vectorOriginal);
   }

   /** {@inheritDoc} */
   @Override
   default void inverseTransform(Orientation3DReadOnly orientationOriginal, Orientation3DBasics orientationTransformed)
   {
      if (hasRotation())
         getRotation().inverseTransform(orientationOriginal, orientationTransformed);
      else
         orientationTransformed.set(orientationOriginal);
   }

   /** {@inheritDoc} */
   @Override
   default void inverseTransform(Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
   {
      vectorTransformed.set(vectorOriginal);
      if (hasTranslation())
      {
         vectorTransformed.subX(vectorTransformed.getS() * getTranslation().getX());
         vectorTransformed.subY(vectorTransformed.getS() * getTranslation().getY());
         vectorTransformed.subZ(vectorTransformed.getS() * getTranslation().getZ());
      }
      if (hasRotation())
         getRotation().inverseTransform(vectorTransformed, vectorTransformed);
   }

   /** {@inheritDoc} */
   @Override
   default void inverseTransform(Point2DReadOnly point2DOriginal, Point2DBasics point2DTransformed, boolean checkIfTransformInXYPlane)
   {
      point2DTransformed.set(point2DOriginal);
      if (hasTranslation())
         point2DTransformed.sub(getTranslation().getX(), getTranslation().getY());
      if (hasRotation())
         getRotation().inverseTransform(point2DTransformed, checkIfTransformInXYPlane);
   }

   /** {@inheritDoc} */
   @Override
   default void inverseTransform(Vector2DReadOnly vector2DOriginal, Vector2DBasics vector2DTransformed, boolean checkIfTransformInXYPlane)
   {
      if (hasRotation())
         getRotation().inverseTransform(vector2DOriginal, vector2DTransformed, checkIfTransformInXYPlane);
      else
         vector2DTransformed.set(vector2DOriginal);
   }

   /** {@inheritDoc} */
   @Override
   default void inverseTransform(Matrix3DReadOnly matrixOriginal, Matrix3DBasics matrixTransformed)
   {
      if (hasRotation())
         getRotation().inverseTransform(matrixOriginal, matrixTransformed);
      else
         matrixTransformed.set(matrixOriginal);
   }

   /** {@inheritDoc} */
   @Override
   default void inverseTransform(RigidBodyTransformReadOnly original, RigidBodyTransformBasics transformed)
   {
      transformed.set(original);
      transformed.preMultiplyInvertOther(this);
   }

   /** {@inheritDoc} */
   @Override
   default void inverseTransform(AffineTransformReadOnly original, AffineTransformBasics transformed)
   {
      transformed.set(original);
      transformed.preMultiplyInvertOther(this);
   }

   /**
    * Gets the x-component of the translation part of this transform.
    *
    * @return the x-component of the translation part.
    */
   default double getTranslationX()
   {
      return getTranslation().getX();
   }

   /**
    * Gets the y-component of the translation part of this transform.
    *
    * @return the y-component of the translation part.
    */
   default double getTranslationY()
   {
      return getTranslation().getY();
   }

   /**
    * Gets the z-component of the translation part of this transform.
    *
    * @return the z-component of the translation part.
    */
   default double getTranslationZ()
   {
      return getTranslation().getZ();
   }

   /**
    * Packs the rotation matrix and translation vector of this rigid-body transform.
    *
    * @param orientationToPack the orientation to set to the rotation of this transform. Modified.
    * @param translationToPack the tuple to set to the translation of this transform. Modified.
    */
   default void get(Orientation3DBasics orientationToPack, Tuple3DBasics translationToPack)
   {
      orientationToPack.set(getRotation());
      translationToPack.set(getTranslation());
   }

   /**
    * Packs the rotation matrix and translation vector of this rigid-body transform.
    * <p>
    * WARNING: a rotation vector is different from a yaw-pitch-roll or Euler angles representation. A
    * rotation vector is equivalent to the axis of an axis-angle that is multiplied by the angle of the
    * same axis-angle.
    * </p>
    *
    * @param rotationVectorToPack the rotation vector to set to the rotation of this transform.
    *                             Modified.
    * @param translationToPack    the tuple to set to the translation of this transform. Modified.
    */
   default void get(Vector3DBasics rotationVectorToPack, Tuple3DBasics translationToPack)
   {
      getRotation().getRotationVector(rotationVectorToPack);
      translationToPack.set(getTranslation());
   }

   /**
    * Packs the rotation matrix and translation vector of this rigid-body transform.
    *
    * @param rotationMatrixToPack the matrix to set to the rotation of this transform. Modified.
    * @param translationToPack    the tuple to set to the translation of this transform. Modified.
    */
   default void get(RotationMatrixBasics rotationMatrixToPack, Tuple3DBasics translationToPack)
   {
      rotationMatrixToPack.set(getRotation());
      translationToPack.set(getTranslation());
   }

   /**
    * Tests separately and on a per component basis if the rotation part and the translation part of
    * this transform and {@code other} are equal to an {@code epsilon}.
    *
    * @param object  the object to compare against this.
    * @param epsilon tolerance to use when comparing each component.
    * @return {@code true} if the two objects are equal component-wise, {@code false} otherwise.
    */
   @Override
   default boolean epsilonEquals(Object object, double epsilon)
   {
      if (!(object instanceof RigidBodyTransformReadOnly))
         return false;
      RigidBodyTransformReadOnly other = (RigidBodyTransformReadOnly) object;
      return getRotation().epsilonEquals(other.getRotation(), epsilon) && getTranslation().epsilonEquals(other.getTranslation(), epsilon);
   }

   /**
    * Two rigid body transforms are considered geometrically equal if both the rotation matrices and
    * translation vectors are equal.
    *
    * @param object  the object to compare against this.
    * @param epsilon the tolerance to use when comparing each component.
    * @return {@code true} if the two rigid body transforms are equal, {@code false} otherwise.
    */
   @Override
   default boolean geometricallyEquals(Object object, double epsilon)
   {
      if (!(object instanceof RigidBodyTransformReadOnly))
         return false;
      RigidBodyTransformReadOnly other = (RigidBodyTransformReadOnly) object;
      return other.getRotation().geometricallyEquals(getRotation(), epsilon) && other.getTranslation().geometricallyEquals(getTranslation(), epsilon);
   }

   /**
    * Gets a representative {@code String} of {@code rigidBodyTransform} given a specific format to
    * use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    *  0.596  0.630  0.930 | -0.435
    * -0.264  0.763  0.575 | -0.464
    * -0.430 -0.188 -0.048 |  0.611
    *  0.000  0.000  0.000 |  1.000
    * </pre>
    * </p>
    *
    * @param format the format to use for each number.
    * @return the representative {@code String}.
    */
   @Override
   default String toString(String format)
   {
      return EuclidCoreIOTools.getRigidBodyTransformString(format, this);
   }
}
