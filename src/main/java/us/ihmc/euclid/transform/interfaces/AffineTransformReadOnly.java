package us.ihmc.euclid.transform.interfaces;

import org.ejml.data.DMatrix;

import us.ihmc.euclid.interfaces.EuclidGeometry;
import us.ihmc.euclid.matrix.interfaces.CommonMatrix3DBasics;
import us.ihmc.euclid.matrix.interfaces.LinearTransform3DReadOnly;
import us.ihmc.euclid.matrix.interfaces.Matrix3DBasics;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DBasics;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.TupleTools;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DBasics;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DReadOnly;

/**
 * Read-only interface for an affine transform.
 * <p>
 * An affine transform represents a transform that can rotate, scale, shear, and/or translate
 * geometries.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface AffineTransformReadOnly extends Transform
{
   /**
    * Gets the read-only reference of the linear part of this transform, such as rotation and scaling.
    *
    * @return the read-only reference of the linear part of this transform.
    */
   LinearTransform3DReadOnly getLinearTransform();

   /**
    * Gets the read-only reference of the translation part of this transform.
    *
    * @return the translation part of this transform.
    */
   Vector3DReadOnly getTranslation();

   /**
    * Requests whether this transform's linear part is to be considered or not.
    * <p>
    * If the linear part of the transform is equal to the identity matrix, it can then be ignored when
    * performing operations such as multiplications and transforms.
    * </p>
    *
    * @return {@code true} if the linear part is non-negligible, {@code false} if the rotation part is
    *         equal to the identity matrix and can be ignored when transforming an object.
    */
   default boolean hasLinearTransform()
   {
      return !getLinearTransform().isIdentity();
   }

   /**
    * Requests whether this transform has a non-zero translation or not.
    *
    * @return {@code true} if the translation part is not zero, {@code false} if the translation part
    *         is zero and can be ignore when transforming an object.
    */
   default boolean hasTranslation()
   {
      return !TupleTools.isTupleZero(getTranslation(), RigidBodyTransformReadOnly.EPS_ZERO_TRANSLATION);
   }

   /** {@inheritDoc} */
   @Override
   default void transform(Point3DReadOnly pointOriginal, Point3DBasics pointTransformed)
   {
      if (hasLinearTransform())
         getLinearTransform().transform(pointOriginal, pointTransformed);
      else
         pointTransformed.set(pointOriginal);

      if (hasTranslation())
         pointTransformed.add(getTranslation());
   }

   /** {@inheritDoc} */
   @Override
   default void transform(Vector3DReadOnly vectorOriginal, Vector3DBasics vectorTransformed)
   {
      if (hasLinearTransform())
         getLinearTransform().transform(vectorOriginal, vectorTransformed);
      else
         vectorTransformed.set(vectorOriginal);
   }

   /** {@inheritDoc} */
   @Override
   default void transform(Orientation3DReadOnly orientationOriginal, Orientation3DBasics orientationTransformed)
   {
      if (hasLinearTransform())
         getLinearTransform().transform(orientationOriginal, orientationTransformed);
      else
         orientationTransformed.set(orientationOriginal);
   }

   /** {@inheritDoc} */
   @Override
   default void transform(Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
   {
      if (hasLinearTransform())
         getLinearTransform().transform(vectorOriginal, vectorTransformed);
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
      if (hasLinearTransform())
         getLinearTransform().transform(point2DOriginal, point2DTransformed, checkIfTransformInXYPlane);
      else
         point2DTransformed.set(point2DOriginal);
      if (hasTranslation())
         point2DTransformed.add(getTranslation().getX(), getTranslation().getY());
   }

   /** {@inheritDoc} */
   @Override
   default void transform(Vector2DReadOnly vector2DOriginal, Vector2DBasics vector2DTransformed, boolean checkIfTransformInXYPlane)
   {
      if (hasLinearTransform())
         getLinearTransform().transform(vector2DOriginal, vector2DTransformed, checkIfTransformInXYPlane);
      else
         vector2DTransformed.set(vector2DOriginal);
   }

   /** {@inheritDoc} */
   @Override
   default void transform(Matrix3DReadOnly matrixOriginal, Matrix3DBasics matrixTransformed)
   {
      if (hasLinearTransform())
         getLinearTransform().transform(matrixOriginal, matrixTransformed);
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
      if (hasLinearTransform())
         getLinearTransform().inverseTransform(pointTransformed);
   }

   /** {@inheritDoc} */
   @Override
   default void inverseTransform(Vector3DReadOnly vectorOriginal, Vector3DBasics vectorTransformed)
   {
      if (hasLinearTransform())
         getLinearTransform().inverseTransform(vectorOriginal, vectorTransformed);
      else
         vectorTransformed.set(vectorOriginal);
   }

   /** {@inheritDoc} */
   @Override
   default void inverseTransform(Orientation3DReadOnly orientationOriginal, Orientation3DBasics orientationTransformed)
   {
      if (hasLinearTransform())
         getLinearTransform().inverseTransform(orientationOriginal, orientationTransformed);
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
      if (hasLinearTransform())
         getLinearTransform().inverseTransform(vectorTransformed, vectorTransformed);
   }

   /** {@inheritDoc} */
   @Override
   default void inverseTransform(Point2DReadOnly point2DOriginal, Point2DBasics point2DTransformed, boolean checkIfTransformInXYPlane)
   {
      point2DTransformed.set(point2DOriginal);
      if (hasTranslation())
         point2DTransformed.sub(getTranslation().getX(), getTranslation().getY());
      if (hasLinearTransform())
         getLinearTransform().inverseTransform(point2DTransformed, checkIfTransformInXYPlane);
   }

   /** {@inheritDoc} */
   @Override
   default void inverseTransform(Vector2DReadOnly vector2DOriginal, Vector2DBasics vector2DTransformed, boolean checkIfTransformInXYPlane)
   {
      if (hasLinearTransform())
         getLinearTransform().inverseTransform(vector2DOriginal, vector2DTransformed, checkIfTransformInXYPlane);
      else
         vector2DTransformed.set(vector2DOriginal);
   }

   /** {@inheritDoc} */
   @Override
   default void inverseTransform(Matrix3DReadOnly matrixOriginal, Matrix3DBasics matrixTransformed)
   {
      if (hasLinearTransform())
         getLinearTransform().inverseTransform(matrixOriginal, matrixTransformed);
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
    * Packs this transform as a 4-by-4 matrix.
    *
    * <pre>
    *     / M(0, 0) M(0, 1) M(0, 2) Tx \
    * H = | M(1, 0) M(1, 1) M(1, 2) Ty |
    *     | M(2, 0) M(2, 1) M(2, 2) Tz |
    *     \    0       0       0     1 /
    * </pre>
    *
    * where M is the 3-by-3 rotation-scale matrix and (Tx, Ty, Tz) is the translation part of this
    * transform.
    *
    * @param matrixToPack the matrix in which this transform is stored. Modified.
    */
   default void get(DMatrix matrixToPack)
   {
      EuclidCoreTools.checkMatrixMinimumSize(4, 4, matrixToPack);
      getLinearTransform().get(matrixToPack);
      getTranslation().get(0, 3, matrixToPack);
      matrixToPack.unsafe_set(3, 0, 0.0);
      matrixToPack.unsafe_set(3, 1, 0.0);
      matrixToPack.unsafe_set(3, 2, 0.0);
      matrixToPack.unsafe_set(3, 3, 1.0);
   }

   /**
    * Packs this transform as a 4-by-4 matrix.
    *
    * <pre>
    *     / M(0, 0) M(0, 1) M(0, 2) Tx \
    * H = | M(1, 0) M(1, 1) M(1, 2) Ty |
    *     | M(2, 0) M(2, 1) M(2, 2) Tz |
    *     \    0       0       0     1 /
    * </pre>
    *
    * where M is the 3-by-3 rotation-scale matrix and (Tx, Ty, Tz) is the translation part of this
    * transform.
    *
    * @param startRow     the first row index to start writing in {@code matrixToPack}.
    * @param startColumn  the first column index to start writing in {@code matrixToPack}.
    * @param matrixToPack the matrix in which this transform is stored. Modified.
    */
   default void get(int startRow, int startColumn, DMatrix matrixToPack)
   {
      EuclidCoreTools.checkMatrixMinimumSize(startRow + 4, startColumn + 4, matrixToPack);
      getLinearTransform().get(startRow, startColumn, matrixToPack);
      getTranslation().get(startRow, startColumn + 3, matrixToPack);
      startRow += 3;
      matrixToPack.unsafe_set(startRow, startColumn++, 0.0);
      matrixToPack.unsafe_set(startRow, startColumn++, 0.0);
      matrixToPack.unsafe_set(startRow, startColumn++, 0.0);
      matrixToPack.unsafe_set(startRow, startColumn, 1.0);
   }

   /**
    * Packs this transform as a 4-by-4 matrix into a 1D row-major array.
    *
    * <pre>
    *     / M(0, 0) M(0, 1) M(0, 2) Tx \
    * H = | M(1, 0) M(1, 1) M(1, 2) Ty |
    *     | M(2, 0) M(2, 1) M(2, 2) Tz |
    *     \    0       0       0     1 /
    * </pre>
    *
    * where M is the 3-by-3 rotation-scale matrix and (Tx, Ty, Tz) is the translation part of this
    * transform.
    *
    * @param transformArrayToPack the array in which this transform is stored. Modified.
    */
   default void get(double[] transformArrayToPack)
   {
      transformArrayToPack[0] = getLinearTransform().getM00();
      transformArrayToPack[1] = getLinearTransform().getM01();
      transformArrayToPack[2] = getLinearTransform().getM02();
      transformArrayToPack[3] = getTranslation().getX();
      transformArrayToPack[4] = getLinearTransform().getM10();
      transformArrayToPack[5] = getLinearTransform().getM11();
      transformArrayToPack[6] = getLinearTransform().getM12();
      transformArrayToPack[7] = getTranslation().getY();
      transformArrayToPack[8] = getLinearTransform().getM20();
      transformArrayToPack[9] = getLinearTransform().getM21();
      transformArrayToPack[10] = getLinearTransform().getM22();
      transformArrayToPack[11] = getTranslation().getZ();
      transformArrayToPack[12] = 0.0;
      transformArrayToPack[13] = 0.0;
      transformArrayToPack[14] = 0.0;
      transformArrayToPack[15] = 1.0;
   }

   /**
    * Packs the rotation-scale matrix and the translation vector of this affine transform.
    *
    * @param rotationScaleMarixToPack matrix in which the rotation-scale matrix of this affine
    *                                 transform is stored. Modified.
    * @param translationToPack        tuple in which the translation vector of this affine transform is
    *                                 stored. Modified.
    */
   default void get(CommonMatrix3DBasics rotationScaleMarixToPack, Tuple3DBasics translationToPack)
   {
      rotationScaleMarixToPack.set(getLinearTransform());
      translationToPack.set(getTranslation());
   }

   /**
    * Two affine transforms are considered geometrically equal if both the linear transform and
    * translation vector are geometrically equal. Returns false by default if incoming object is not a
    * type of AffineTransform.
    *
    * @param geometry the object to compare against this.
    * @param epsilon  the tolerance to use when comparing each component.
    * @return {@code true} if the two transforms are equal, {@code false} otherwise.
    */
   @Override
   default boolean geometricallyEquals(EuclidGeometry geometry, double epsilon)
   {
      if (!(geometry instanceof AffineTransformReadOnly))
         return false;

      AffineTransformReadOnly other = (AffineTransformReadOnly) geometry;
      return getLinearTransform().geometricallyEquals(other.getLinearTransform(), epsilon)
            && getTranslation().geometricallyEquals(other.getTranslation(), epsilon);
   }

   /** {@inheritDoc} */
   @Override
   default boolean epsilonEquals(EuclidGeometry geometry, double epsilon)
   {
      if (geometry == this)
         return true;
      if (geometry == null)
         return false;
      if (!(geometry instanceof AffineTransformReadOnly))
         return false;

      AffineTransformReadOnly other = (AffineTransformReadOnly) geometry;
      return getLinearTransform().epsilonEquals(other.getLinearTransform(), epsilon) && getTranslation().epsilonEquals(other.getTranslation(), epsilon);
   }

   /** {@inheritDoc} */
   @Override
   default boolean equals(EuclidGeometry geometry)
   {
      if (geometry == this)
         return true;
      if (geometry == null)
         return false;
      else if (!(geometry instanceof AffineTransformReadOnly))
         return false;
      AffineTransformReadOnly other = (AffineTransformReadOnly) geometry;
      return getLinearTransform().equals(other.getLinearTransform()) && getTranslation().equals(other.getTranslation());
   }

   @Override
   default String toString(String format)
   {
      return EuclidCoreIOTools.getAffineTransformString(format, this);
   }
}
