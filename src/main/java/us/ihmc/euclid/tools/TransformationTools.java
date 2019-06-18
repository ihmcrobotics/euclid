package us.ihmc.euclid.tools;

import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.euclid.transform.QuaternionBasedTransform;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

/**
 * {@code TransformationTools} provides a list a methods for transforming geometry objects useful in
 * particular contexts where the result cannot be stored in an object.
 * <p>
 * Note that in common situations, the use of {@code TransformationTools} should be avoided
 * preferring the use of the 'transform' or 'applyTransform' methods provided with the concerned
 * objects. Also note that these methods are possibly more computationally expensive than their
 * respective counterparts.
 * </p>
 *
 * @deprecated This tools class will be removed in a future release.
 * @author Sylvain Bertrand
 */
@Deprecated
public class TransformationTools
{
   private TransformationTools()
   {
      // Suppresses default constructor, ensuring non-instantiability.
   }

   /**
    * Computes and returns the x-coordinate resulting from the transformation of {@code tupleOriginal}
    * by {@code matrix}.
    *
    * @param matrix        the matrix used to transform the given tuple. Not modified.
    * @param transpose     whether the operation should performed with the transpose of the given
    *                      matrix or not.
    * @param tupleOriginal the tuple to be transformed. Not modified.
    * @return the x-coordinate resulting from the transformation.
    */
   public static double computeTransformedX(Matrix3DReadOnly matrix, boolean transpose, Tuple3DReadOnly tupleOriginal)
   {
      return computeTransformedX(matrix, transpose, tupleOriginal.getX(), tupleOriginal.getY(), tupleOriginal.getZ());
   }

   /**
    * Computes and returns the x-coordinate resulting from the transformation of {@code tupleOriginal}
    * by {@code matrix}.
    *
    * @param matrix    the matrix used to transform the given tuple. Not modified.
    * @param transpose whether the operation should performed with the transpose of the given matrix or
    *                  not.
    * @param x         the x-coordinate of the tuple to be transformed.
    * @param y         the y-coordinate of the tuple to be transformed.
    * @param z         the z-coordinate of the tuple to be transformed.
    * @return the x-coordinate resulting from the transformation.
    */
   public static double computeTransformedX(Matrix3DReadOnly matrix, boolean transpose, double x, double y, double z)
   {
      if (transpose)
         return matrix.getM00() * x + matrix.getM10() * y + matrix.getM20() * z;
      else
         return matrix.getM00() * x + matrix.getM01() * y + matrix.getM02() * z;
   }

   /**
    * Computes and returns the y-coordinate resulting from the transformation of {@code tupleOriginal}
    * by {@code matrix}.
    *
    * @param matrix        the matrix used to transform the given tuple. Not modified.
    * @param transpose     whether the operation should performed with the transpose of the given
    *                      matrix or not.
    * @param tupleOriginal the tuple to be transformed. Not modified.
    * @return the y-coordinate resulting from the transformation.
    */
   public static double computeTransformedY(Matrix3DReadOnly matrix, boolean transpose, Tuple3DReadOnly tupleOriginal)
   {
      return computeTransformedY(matrix, transpose, tupleOriginal.getX(), tupleOriginal.getY(), tupleOriginal.getZ());
   }

   /**
    * Computes and returns the y-coordinate resulting from the transformation of {@code tupleOriginal}
    * by {@code matrix}.
    *
    * @param matrix    the matrix used to transform the given tuple. Not modified.
    * @param transpose whether the operation should performed with the transpose of the given matrix or
    *                  not.
    * @param x         the x-coordinate of the tuple to be transformed.
    * @param y         the y-coordinate of the tuple to be transformed.
    * @param z         the z-coordinate of the tuple to be transformed.
    * @return the y-coordinate resulting from the transformation.
    */
   public static double computeTransformedY(Matrix3DReadOnly matrix, boolean transpose, double x, double y, double z)
   {
      if (transpose)
         return matrix.getM01() * x + matrix.getM11() * y + matrix.getM21() * z;
      else
         return matrix.getM10() * x + matrix.getM11() * y + matrix.getM12() * z;
   }

   /**
    * Computes and returns the z-coordinate resulting from the transformation of {@code tupleOriginal}
    * by {@code matrix}.
    *
    * @param matrix        the matrix used to transform the given tuple. Not modified.
    * @param transpose     whether the operation should performed with the transpose of the given
    *                      matrix or not.
    * @param tupleOriginal the tuple to be transformed. Not modified.
    * @return the z-coordinate resulting from the transformation.
    */
   public static double computeTransformedZ(Matrix3DReadOnly matrix, boolean transpose, Tuple3DReadOnly tupleOriginal)
   {
      return computeTransformedZ(matrix, transpose, tupleOriginal.getX(), tupleOriginal.getY(), tupleOriginal.getZ());
   }

   /**
    * Computes and returns the z-coordinate resulting from the transformation of {@code tupleOriginal}
    * by {@code matrix}.
    *
    * @param matrix    the matrix used to transform the given tuple. Not modified.
    * @param transpose whether the operation should performed with the transpose of the given matrix or
    *                  not.
    * @param x         the x-coordinate of the tuple to be transformed.
    * @param y         the y-coordinate of the tuple to be transformed.
    * @param z         the z-coordinate of the tuple to be transformed.
    * @return the z-coordinate resulting from the transformation.
    */
   public static double computeTransformedZ(Matrix3DReadOnly matrix, boolean transpose, double x, double y, double z)
   {
      if (transpose)
         return matrix.getM02() * x + matrix.getM12() * y + matrix.getM22() * z;
      else
         return matrix.getM20() * x + matrix.getM21() * y + matrix.getM22() * z;
   }

   /**
    * Computes and returns the x-coordinate resulting from the transformation of {@code tupleOriginal}
    * by {@code quaternion}.
    *
    * @param quaternion    the quaternion used to transform the given tuple. Not modified.
    * @param conjugate     whether the operation should performed with the conjugate of the given
    *                      quaternion or not.
    * @param tupleOriginal the tuple to be transformed. Not modified.
    * @return the x-coordinate resulting from the transformation.
    */
   public static double computeTransformedX(QuaternionReadOnly quaternion, boolean conjugate, Tuple3DReadOnly tupleOriginal)
   {
      return computeTransformedX(quaternion, conjugate, tupleOriginal.getX(), tupleOriginal.getY(), tupleOriginal.getZ());
   }

   /**
    * Computes and returns the x-coordinate resulting from the transformation of {@code tupleOriginal}
    * by {@code quaternion}.
    *
    * @param quaternion the quaternion used to transform the given tuple. Not modified.
    * @param conjugate  whether the operation should performed with the conjugate of the given
    *                   quaternion or not.
    * @param x          the x-coordinate of the tuple to be transformed.
    * @param y          the y-coordinate of the tuple to be transformed.
    * @param z          the z-coordinate of the tuple to be transformed.
    * @return the x-coordinate resulting from the transformation.
    */
   public static double computeTransformedX(QuaternionReadOnly quaternion, boolean conjugate, double x, double y, double z)
   {
      double qx = quaternion.getX();
      double qy = quaternion.getY();
      double qz = quaternion.getZ();
      double qs = quaternion.getS();

      if (conjugate)
      {
         qx = -qx;
         qy = -qy;
         qz = -qz;
      }

      double norm = quaternion.norm();

      if (norm < QuaternionTools.EPS)
      {
         return x;
      }

      norm = 1.0 / norm;
      double crossX = qy * z - qz * y;
      double crossY = qz * x - qx * z;
      double crossZ = qx * y - qy * x;

      return x + 2.0 * (qs * crossX + qy * crossZ - qz * crossY) * norm * norm;
   }

   /**
    * Computes and returns the y-coordinate resulting from the transformation of {@code tupleOriginal}
    * by {@code quaternion}.
    *
    * @param quaternion    the quaternion used to transform the given tuple. Not modified.
    * @param conjugate     whether the operation should performed with the conjugate of the given
    *                      quaternion or not.
    * @param tupleOriginal the tuple to be transformed. Not modified.
    * @return the y-coordinate resulting from the transformation.
    */
   public static double computeTransformedY(QuaternionReadOnly quaternion, boolean conjugate, Tuple3DReadOnly tupleOriginal)
   {
      return computeTransformedY(quaternion, conjugate, tupleOriginal.getX(), tupleOriginal.getY(), tupleOriginal.getZ());
   }

   /**
    * Computes and returns the y-coordinate resulting from the transformation of {@code tupleOriginal}
    * by {@code quaternion}.
    *
    * @param quaternion the quaternion used to transform the given tuple. Not modified.
    * @param conjugate  whether the operation should performed with the conjugate of the given
    *                   quaternion or not.
    * @param x          the x-coordinate of the tuple to be transformed.
    * @param y          the y-coordinate of the tuple to be transformed.
    * @param z          the z-coordinate of the tuple to be transformed.
    * @return the y-coordinate resulting from the transformation.
    */
   public static double computeTransformedY(QuaternionReadOnly quaternion, boolean conjugate, double x, double y, double z)
   {
      double qx = quaternion.getX();
      double qy = quaternion.getY();
      double qz = quaternion.getZ();
      double qs = quaternion.getS();

      if (conjugate)
      {
         qx = -qx;
         qy = -qy;
         qz = -qz;
      }

      double norm = quaternion.norm();

      if (norm < QuaternionTools.EPS)
      {
         return y;
      }

      norm = 1.0 / norm;
      double crossX = qy * z - qz * y;
      double crossY = qz * x - qx * z;
      double crossZ = qx * y - qy * x;

      return y + 2.0 * (qs * crossY + qz * crossX - qx * crossZ) * norm * norm;
   }

   /**
    * Computes and returns the z-coordinate resulting from the transformation of {@code tupleOriginal}
    * by {@code quaternion}.
    *
    * @param quaternion    the quaternion used to transform the given tuple. Not modified.
    * @param conjugate     whether the operation should performed with the conjugate of the given
    *                      quaternion or not.
    * @param tupleOriginal the tuple to be transformed. Not modified.
    * @return the z-coordinate resulting from the transformation.
    */
   public static double computeTransformedZ(QuaternionReadOnly quaternion, boolean conjugate, Tuple3DReadOnly tupleOriginal)
   {
      return computeTransformedZ(quaternion, conjugate, tupleOriginal.getX(), tupleOriginal.getY(), tupleOriginal.getZ());
   }

   /**
    * Computes and returns the z-coordinate resulting from the transformation of {@code tupleOriginal}
    * by {@code quaternion}.
    *
    * @param quaternion the quaternion used to transform the given tuple. Not modified.
    * @param conjugate  whether the operation should performed with the conjugate of the given
    *                   quaternion or not.
    * @param x          the x-coordinate of the tuple to be transformed.
    * @param y          the y-coordinate of the tuple to be transformed.
    * @param z          the z-coordinate of the tuple to be transformed.
    * @return the z-coordinate resulting from the transformation.
    */
   public static double computeTransformedZ(QuaternionReadOnly quaternion, boolean conjugate, double x, double y, double z)
   {
      double qx = quaternion.getX();
      double qy = quaternion.getY();
      double qz = quaternion.getZ();
      double qs = quaternion.getS();

      if (conjugate)
      {
         qx = -qx;
         qy = -qy;
         qz = -qz;
      }

      double norm = quaternion.norm();

      if (norm < QuaternionTools.EPS)
      {
         return z;
      }

      norm = 1.0 / norm;
      double crossX = qy * z - qz * y;
      double crossY = qz * x - qx * z;
      double crossZ = qx * y - qy * x;

      return z + 2.0 * (qs * crossZ + qx * crossY - qy * crossX) * norm * norm;
   }

   /**
    * Computes and returns the x-coordinate resulting from the transformation of {@code pointOriginal}
    * by {@code rigidBodyTransform}.
    *
    * @param rigidBodyTransform the transform used to transform the given point. Not modified.
    * @param invert             whether the operation should performed with the inverse of the given
    *                           transform or not.
    * @param pointOriginal      the point to be transformed. Not modified.
    * @return the x-coordinate resulting from the transformation.
    */
   public static double computeTransformedX(RigidBodyTransform rigidBodyTransform, boolean invert, Point3DReadOnly pointOriginal)
   {
      if (invert)
      {
         double x = pointOriginal.getX() - rigidBodyTransform.getTranslationX();
         double y = pointOriginal.getY() - rigidBodyTransform.getTranslationY();
         double z = pointOriginal.getZ() - rigidBodyTransform.getTranslationZ();
         return computeTransformedX(rigidBodyTransform.getRotation(), invert, x, y, z);
      }
      else
      {
         return rigidBodyTransform.getTranslationX() + computeTransformedX(rigidBodyTransform.getRotation(), invert, pointOriginal);
      }
   }

   /**
    * Computes and returns the y-coordinate resulting from the transformation of {@code pointOriginal}
    * by {@code rigidBodyTransform}.
    *
    * @param rigidBodyTransform the transform used to transform the given point. Not modified.
    * @param invert             whether the operation should performed with the inverse of the given
    *                           transform or not.
    * @param pointOriginal      the point to be transformed. Not modified.
    * @return the y-coordinate resulting from the transformation.
    */
   public static double computeTransformedY(RigidBodyTransform rigidBodyTransform, boolean invert, Point3DReadOnly pointOriginal)
   {
      if (invert)
      {
         double x = pointOriginal.getX() - rigidBodyTransform.getTranslationX();
         double y = pointOriginal.getY() - rigidBodyTransform.getTranslationY();
         double z = pointOriginal.getZ() - rigidBodyTransform.getTranslationZ();
         return computeTransformedY(rigidBodyTransform.getRotation(), invert, x, y, z);
      }
      else
      {
         return rigidBodyTransform.getTranslationY() + computeTransformedY(rigidBodyTransform.getRotation(), invert, pointOriginal);
      }
   }

   /**
    * Computes and returns the z-coordinate resulting from the transformation of {@code pointOriginal}
    * by {@code rigidBodyTransform}.
    *
    * @param rigidBodyTransform the transform used to transform the given point. Not modified.
    * @param invert             whether the operation should performed with the inverse of the given
    *                           transform or not.
    * @param pointOriginal      the point to be transformed. Not modified.
    * @return the z-coordinate resulting from the transformation.
    */
   public static double computeTransformedZ(RigidBodyTransform rigidBodyTransform, boolean invert, Point3DReadOnly pointOriginal)
   {
      if (invert)
      {
         double x = pointOriginal.getX() - rigidBodyTransform.getTranslationX();
         double y = pointOriginal.getY() - rigidBodyTransform.getTranslationY();
         double z = pointOriginal.getZ() - rigidBodyTransform.getTranslationZ();
         return computeTransformedZ(rigidBodyTransform.getRotation(), invert, x, y, z);
      }
      else
      {
         return rigidBodyTransform.getTranslationZ() + computeTransformedZ(rigidBodyTransform.getRotation(), invert, pointOriginal);
      }
   }

   /**
    * Computes and returns the x-coordinate resulting from the transformation of {@code vectorOriginal}
    * by {@code rigidBodyTransform}.
    *
    * @param rigidBodyTransform the transform used to transform the given vector. Not modified.
    * @param invert             whether the operation should performed with the inverse of the given
    *                           transform or not.
    * @param vectorOriginal     the vector to be transformed. Not modified.
    * @return the x-coordinate resulting from the transformation.
    */
   public static double computeTransformedX(RigidBodyTransform rigidBodyTransform, boolean invert, Vector3DReadOnly vectorOriginal)
   {
      return computeTransformedX(rigidBodyTransform.getRotation(), invert, vectorOriginal);
   }

   /**
    * Computes and returns the y-coordinate resulting from the transformation of {@code vectorOriginal}
    * by {@code rigidBodyTransform}.
    *
    * @param rigidBodyTransform the transform used to transform the given vector. Not modified.
    * @param invert             whether the operation should performed with the inverse of the given
    *                           transform or not.
    * @param vectorOriginal     the vector to be transformed. Not modified.
    * @return the y-coordinate resulting from the transformation.
    */
   public static double computeTransformedY(RigidBodyTransform rigidBodyTransform, boolean invert, Vector3DReadOnly vectorOriginal)
   {
      return computeTransformedY(rigidBodyTransform.getRotation(), invert, vectorOriginal);
   }

   /**
    * Computes and returns the z-coordinate resulting from the transformation of {@code vectorOriginal}
    * by {@code rigidBodyTransform}.
    *
    * @param rigidBodyTransform the transform used to transform the given vector. Not modified.
    * @param invert             whether the operation should performed with the inverse of the given
    *                           transform or not.
    * @param vectorOriginal     the vector to be transformed. Not modified.
    * @return the z-coordinate resulting from the transformation.
    */
   public static double computeTransformedZ(RigidBodyTransform rigidBodyTransform, boolean invert, Vector3DReadOnly vectorOriginal)
   {
      return computeTransformedZ(rigidBodyTransform.getRotation(), invert, vectorOriginal);
   }

   /**
    * Computes and returns the x-coordinate resulting from the transformation of {@code pointOriginal}
    * by {@code quaternionBasedTransform}.
    *
    * @param quaternionBasedTransform the transform used to transform the given point. Not modified.
    * @param invert                   whether the operation should performed with the inverse of the
    *                                 given transform or not.
    * @param pointOriginal            the point to be transformed. Not modified.
    * @return the x-coordinate resulting from the transformation.
    */
   public static double computeTransformedX(QuaternionBasedTransform quaternionBasedTransform, boolean invert, Point3DReadOnly pointOriginal)
   {
      if (invert)
      {
         double x = pointOriginal.getX() - quaternionBasedTransform.getTranslationX();
         double y = pointOriginal.getY() - quaternionBasedTransform.getTranslationY();
         double z = pointOriginal.getZ() - quaternionBasedTransform.getTranslationZ();
         return computeTransformedX(quaternionBasedTransform.getRotation(), invert, x, y, z);
      }
      else
      {
         return quaternionBasedTransform.getTranslationX() + computeTransformedX(quaternionBasedTransform.getRotation(), invert, pointOriginal);
      }
   }

   /**
    * Computes and returns the y-coordinate resulting from the transformation of {@code pointOriginal}
    * by {@code quaternionBasedTransform}.
    *
    * @param quaternionBasedTransform the transform used to transform the given point. Not modified.
    * @param invert                   whether the operation should performed with the inverse of the
    *                                 given transform or not.
    * @param pointOriginal            the point to be transformed. Not modified.
    * @return the y-coordinate resulting from the transformation.
    */
   public static double computeTransformedY(QuaternionBasedTransform quaternionBasedTransform, boolean invert, Point3DReadOnly pointOriginal)
   {
      if (invert)
      {
         double x = pointOriginal.getX() - quaternionBasedTransform.getTranslationX();
         double y = pointOriginal.getY() - quaternionBasedTransform.getTranslationY();
         double z = pointOriginal.getZ() - quaternionBasedTransform.getTranslationZ();
         return computeTransformedY(quaternionBasedTransform.getRotation(), invert, x, y, z);
      }
      else
      {
         return quaternionBasedTransform.getTranslationY() + computeTransformedY(quaternionBasedTransform.getRotation(), invert, pointOriginal);
      }
   }

   /**
    * Computes and returns the z-coordinate resulting from the transformation of {@code pointOriginal}
    * by {@code quaternionBasedTransform}.
    *
    * @param quaternionBasedTransform the transform used to transform the given point. Not modified.
    * @param invert                   whether the operation should performed with the inverse of the
    *                                 given transform or not.
    * @param pointOriginal            the point to be transformed. Not modified.
    * @return the z-coordinate resulting from the transformation.
    */
   public static double computeTransformedZ(QuaternionBasedTransform quaternionBasedTransform, boolean invert, Point3DReadOnly pointOriginal)
   {
      if (invert)
      {
         double x = pointOriginal.getX() - quaternionBasedTransform.getTranslationX();
         double y = pointOriginal.getY() - quaternionBasedTransform.getTranslationY();
         double z = pointOriginal.getZ() - quaternionBasedTransform.getTranslationZ();
         return computeTransformedZ(quaternionBasedTransform.getRotation(), invert, x, y, z);
      }
      else
      {
         return quaternionBasedTransform.getTranslationZ() + computeTransformedZ(quaternionBasedTransform.getRotation(), invert, pointOriginal);
      }
   }

   /**
    * Computes and returns the x-coordinate resulting from the transformation of {@code vectorOriginal}
    * by {@code quaternionBasedTransform}.
    *
    * @param quaternionBasedTransform the transform used to transform the given vector. Not modified.
    * @param invert                   whether the operation should performed with the inverse of the
    *                                 given transform or not.
    * @param vectorOriginal           the vector to be transformed. Not modified.
    * @return the x-coordinate resulting from the transformation.
    */
   public static double computeTransformedX(QuaternionBasedTransform quaternionBasedTransform, boolean invert, Vector3DReadOnly vectorOriginal)
   {
      return computeTransformedX(quaternionBasedTransform.getRotation(), invert, vectorOriginal);
   }

   /**
    * Computes and returns the y-coordinate resulting from the transformation of {@code vectorOriginal}
    * by {@code quaternionBasedTransform}.
    *
    * @param quaternionBasedTransform the transform used to transform the given vector. Not modified.
    * @param invert                   whether the operation should performed with the inverse of the
    *                                 given transform or not.
    * @param vectorOriginal           the vector to be transformed. Not modified.
    * @return the y-coordinate resulting from the transformation.
    */
   public static double computeTransformedY(QuaternionBasedTransform quaternionBasedTransform, boolean invert, Vector3DReadOnly vectorOriginal)
   {
      return computeTransformedY(quaternionBasedTransform.getRotation(), invert, vectorOriginal);
   }

   /**
    * Computes and returns the z-coordinate resulting from the transformation of {@code vectorOriginal}
    * by {@code quaternionBasedTransform}.
    *
    * @param quaternionBasedTransform the transform used to transform the given vector. Not modified.
    * @param invert                   whether the operation should performed with the inverse of the
    *                                 given transform or not.
    * @param vectorOriginal           the vector to be transformed. Not modified.
    * @return the z-coordinate resulting from the transformation.
    */
   public static double computeTransformedZ(QuaternionBasedTransform quaternionBasedTransform, boolean invert, Vector3DReadOnly vectorOriginal)
   {
      return computeTransformedZ(quaternionBasedTransform.getRotation(), invert, vectorOriginal);
   }

   /**
    * Computes and returns the x-coordinate resulting from the transformation of {@code pointOriginal}
    * by {@code affineTransform}.
    *
    * @param affineTransform the transform used to transform the given point. Not modified.
    * @param invert          whether the operation should performed with the inverse of the given
    *                        transform or not.
    * @param pointOriginal   the point to be transformed. Not modified.
    * @return the x-coordinate resulting from the transformation.
    */
   public static double computeTransformedX(AffineTransform affineTransform, boolean invert, Point3DReadOnly pointOriginal)
   {
      if (invert)
      {
         double x = pointOriginal.getX() - affineTransform.getTranslationX();
         double y = pointOriginal.getY() - affineTransform.getTranslationY();
         double z = pointOriginal.getZ() - affineTransform.getTranslationZ();
         return computeTransformedX(affineTransform.getRotationMatrix(), invert, x, y, z) / affineTransform.getScaleX();
      }
      else
      {
         double x = pointOriginal.getX() * affineTransform.getScaleX();
         double y = pointOriginal.getY() * affineTransform.getScaleY();
         double z = pointOriginal.getZ() * affineTransform.getScaleZ();
         return affineTransform.getTranslationX() + computeTransformedX(affineTransform.getRotationMatrix(), invert, x, y, z);
      }
   }

   /**
    * Computes and returns the y-coordinate resulting from the transformation of {@code pointOriginal}
    * by {@code affineTransform}.
    *
    * @param affineTransform the transform used to transform the given point. Not modified.
    * @param invert          whether the operation should performed with the inverse of the given
    *                        transform or not.
    * @param pointOriginal   the point to be transformed. Not modified.
    * @return the y-coordinate resulting from the transformation.
    */
   public static double computeTransformedY(AffineTransform affineTransform, boolean invert, Point3DReadOnly pointOriginal)
   {
      if (invert)
      {
         double x = pointOriginal.getX() - affineTransform.getTranslationX();
         double y = pointOriginal.getY() - affineTransform.getTranslationY();
         double z = pointOriginal.getZ() - affineTransform.getTranslationZ();
         return computeTransformedY(affineTransform.getRotationMatrix(), invert, x, y, z) / affineTransform.getScaleY();
      }
      else
      {
         double x = pointOriginal.getX() * affineTransform.getScaleX();
         double y = pointOriginal.getY() * affineTransform.getScaleY();
         double z = pointOriginal.getZ() * affineTransform.getScaleZ();
         return affineTransform.getTranslationY() + computeTransformedY(affineTransform.getRotationMatrix(), invert, x, y, z);
      }
   }

   /**
    * Computes and returns the z-coordinate resulting from the transformation of {@code pointOriginal}
    * by {@code affineTransform}.
    *
    * @param affineTransform the transform used to transform the given point. Not modified.
    * @param invert          whether the operation should performed with the inverse of the given
    *                        transform or not.
    * @param pointOriginal   the point to be transformed. Not modified.
    * @return the z-coordinate resulting from the transformation.
    */
   public static double computeTransformedZ(AffineTransform affineTransform, boolean invert, Point3DReadOnly pointOriginal)
   {
      if (invert)
      {
         double x = pointOriginal.getX() - affineTransform.getTranslationX();
         double y = pointOriginal.getY() - affineTransform.getTranslationY();
         double z = pointOriginal.getZ() - affineTransform.getTranslationZ();
         return computeTransformedZ(affineTransform.getRotationMatrix(), invert, x, y, z) / affineTransform.getScaleZ();
      }
      else
      {
         double x = pointOriginal.getX() * affineTransform.getScaleX();
         double y = pointOriginal.getY() * affineTransform.getScaleY();
         double z = pointOriginal.getZ() * affineTransform.getScaleZ();
         return affineTransform.getTranslationZ() + computeTransformedZ(affineTransform.getRotationMatrix(), invert, x, y, z);
      }
   }

   /**
    * Computes and returns the x-coordinate resulting from the transformation of {@code vectorOriginal}
    * by {@code affineTransform}.
    *
    * @param affineTransform the transform used to transform the given vector. Not modified.
    * @param invert          whether the operation should performed with the inverse of the given
    *                        transform or not.
    * @param vectorOriginal  the vector to be transformed. Not modified.
    * @return the x-coordinate resulting from the transformation.
    */
   public static double computeTransformedX(AffineTransform affineTransform, boolean invert, Vector3DReadOnly vectorOriginal)
   {
      if (invert)
      {
         return computeTransformedX(affineTransform.getRotationMatrix(), invert, vectorOriginal) / affineTransform.getScaleX();
      }
      else
      {
         double x = vectorOriginal.getX() * affineTransform.getScaleX();
         double y = vectorOriginal.getY() * affineTransform.getScaleY();
         double z = vectorOriginal.getZ() * affineTransform.getScaleZ();
         return computeTransformedX(affineTransform.getRotationMatrix(), invert, x, y, z);
      }
   }

   /**
    * Computes and returns the y-coordinate resulting from the transformation of {@code vectorOriginal}
    * by {@code affineTransform}.
    *
    * @param affineTransform the transform used to transform the given vector. Not modified.
    * @param invert          whether the operation should performed with the inverse of the given
    *                        transform or not.
    * @param vectorOriginal  the vector to be transformed. Not modified.
    * @return the y-coordinate resulting from the transformation.
    */
   public static double computeTransformedY(AffineTransform affineTransform, boolean invert, Vector3DReadOnly vectorOriginal)
   {
      if (invert)
      {
         return computeTransformedY(affineTransform.getRotationMatrix(), invert, vectorOriginal) / affineTransform.getScaleY();
      }
      else
      {
         double x = vectorOriginal.getX() * affineTransform.getScaleX();
         double y = vectorOriginal.getY() * affineTransform.getScaleY();
         double z = vectorOriginal.getZ() * affineTransform.getScaleZ();
         return computeTransformedY(affineTransform.getRotationMatrix(), invert, x, y, z);
      }
   }

   /**
    * Computes and returns the z-coordinate resulting from the transformation of {@code vectorOriginal}
    * by {@code affineTransform}.
    *
    * @param affineTransform the transform used to transform the given vector. Not modified.
    * @param invert          whether the operation should performed with the inverse of the given
    *                        transform or not.
    * @param vectorOriginal  the vector to be transformed. Not modified.
    * @return the z-coordinate resulting from the transformation.
    */
   public static double computeTransformedZ(AffineTransform affineTransform, boolean invert, Vector3DReadOnly vectorOriginal)
   {
      if (invert)
      {
         return computeTransformedZ(affineTransform.getRotationMatrix(), invert, vectorOriginal) / affineTransform.getScaleZ();
      }
      else
      {
         double x = vectorOriginal.getX() * affineTransform.getScaleX();
         double y = vectorOriginal.getY() * affineTransform.getScaleY();
         double z = vectorOriginal.getZ() * affineTransform.getScaleZ();
         return computeTransformedZ(affineTransform.getRotationMatrix(), invert, x, y, z);
      }
   }
}
