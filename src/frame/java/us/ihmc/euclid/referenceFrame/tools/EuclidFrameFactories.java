package us.ihmc.euclid.referenceFrame.tools;

import static us.ihmc.euclid.geometry.tools.EuclidGeometryFactories.newObservableBoundingBox2DBasics;
import static us.ihmc.euclid.geometry.tools.EuclidGeometryFactories.newObservableBoundingBox3DBasics;
import static us.ihmc.euclid.tools.EuclidCoreFactories.newLinkedPoint2DReadOnly;
import static us.ihmc.euclid.tools.EuclidCoreFactories.newLinkedPoint3DReadOnly;
import static us.ihmc.euclid.tools.EuclidCoreFactories.newLinkedVector2DReadOnly;
import static us.ihmc.euclid.tools.EuclidCoreFactories.newLinkedVector3DReadOnly;
import static us.ihmc.euclid.tools.EuclidCoreFactories.newNegativeLinkedPoint2D;
import static us.ihmc.euclid.tools.EuclidCoreFactories.newNegativeLinkedPoint3D;
import static us.ihmc.euclid.tools.EuclidCoreFactories.newNegativeLinkedVector2D;
import static us.ihmc.euclid.tools.EuclidCoreFactories.newNegativeLinkedVector3D;
import static us.ihmc.euclid.tools.EuclidCoreFactories.newObservablePoint2DBasics;
import static us.ihmc.euclid.tools.EuclidCoreFactories.newObservablePoint2DReadOnly;
import static us.ihmc.euclid.tools.EuclidCoreFactories.newObservablePoint3DBasics;
import static us.ihmc.euclid.tools.EuclidCoreFactories.newObservablePoint3DReadOnly;
import static us.ihmc.euclid.tools.EuclidCoreFactories.newObservableQuaternionBasics;
import static us.ihmc.euclid.tools.EuclidCoreFactories.newObservableQuaternionReadOnly;
import static us.ihmc.euclid.tools.EuclidCoreFactories.newObservableRotationMatrixBasics;
import static us.ihmc.euclid.tools.EuclidCoreFactories.newObservableRotationMatrixReadOnly;
import static us.ihmc.euclid.tools.EuclidCoreFactories.newObservableUnitVector2DBasics;
import static us.ihmc.euclid.tools.EuclidCoreFactories.newObservableUnitVector2DReadOnly;
import static us.ihmc.euclid.tools.EuclidCoreFactories.newObservableUnitVector3DBasics;
import static us.ihmc.euclid.tools.EuclidCoreFactories.newObservableUnitVector3DReadOnly;
import static us.ihmc.euclid.tools.EuclidCoreFactories.newObservableVector2DBasics;
import static us.ihmc.euclid.tools.EuclidCoreFactories.newObservableVector2DReadOnly;
import static us.ihmc.euclid.tools.EuclidCoreFactories.newObservableVector3DBasics;
import static us.ihmc.euclid.tools.EuclidCoreFactories.newObservableVector3DReadOnly;
import static us.ihmc.euclid.tools.EuclidHashCodeTools.toIntHashCode;

import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.IntConsumer;
import java.util.function.ObjDoubleConsumer;

import us.ihmc.euclid.Axis2D;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.Bound;
import us.ihmc.euclid.geometry.BoundingBox2D;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.interfaces.BoundingBox2DBasics;
import us.ihmc.euclid.geometry.interfaces.BoundingBox3DBasics;
import us.ihmc.euclid.geometry.tools.EuclidGeometryFactories.BoundingBoxChangedListener;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixBasics;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.orientation.Orientation2D;
import us.ihmc.euclid.orientation.interfaces.Orientation2DBasics;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameBoundingBox2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameBoundingBox3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameOrientation2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameQuaternionBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameRotationMatrixBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameUnitVector2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameUnitVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameBoundingBox2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameBoundingBox3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameMatrix3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameRotationMatrixReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameUnitVector2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameUnitVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.UnitVector2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.UnitVector2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.UnitVector2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.UnitVector3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

/**
 * This class provides a varieties of factories to create Euclid frame types.
 *
 * @author Sylvain Bertrand
 */
public class EuclidFrameFactories
{
   private EuclidFrameFactories()
   {
      // Suppresses default constructor, ensuring non-instantiability.
   }

   /**
    * Creates a new frame point that is linked to the {@code originalTuple} as follows:
    *
    * <pre>
    * linkedPoint = scale * originalTuple
    * </pre>
    *
    * where the scale is obtained from the given {@code scaleSupplier}.
    *
    * @param scaleSupplier the supplier to get the scale.
    * @param originalTuple the reference tuple to scale. Not modified.
    * @return the new point linked to {@code originalTuple}.
    */
   public static FramePoint2DReadOnly newLinkedFramePoint2DReadOnly(DoubleSupplier scaleSupplier, FrameTuple2DReadOnly originalTuple)
   {
      return newLinkedFramePoint2DReadOnly(originalTuple, newLinkedPoint2DReadOnly(scaleSupplier, originalTuple));
   }

   /**
    * Creates a new frame vector that is linked to the {@code originalTuple} as follows:
    *
    * <pre>
    * linkedVector = scale * originalTuple
    * </pre>
    *
    * where the scale is obtained from the given {@code scaleSupplier}.
    *
    * @param scaleSupplier the supplier to get the scale.
    * @param originalTuple the reference tuple to scale. Not modified.
    * @return the new vector linked to {@code originalTuple}.
    */
   public static FrameVector2DReadOnly newLinkedFrameVector2DReadOnly(DoubleSupplier scaleSupplier, FrameTuple2DReadOnly originalTuple)
   {
      return newLinkedFrameVector2DReadOnly(originalTuple, newLinkedVector2DReadOnly(scaleSupplier, originalTuple));
   }

   /**
    * Creates a new frame point that is linked to the {@code originalTuple} as follows:
    *
    * <pre>
    * linkedPoint = scale * originalTuple
    * </pre>
    *
    * where the scale is obtained from the given {@code scaleSupplier}.
    *
    * @param scaleSupplier the supplier to get the scale.
    * @param originalTuple the reference tuple to scale. Not modified.
    * @return the new point linked to {@code originalTuple}.
    */
   public static FramePoint3DReadOnly newLinkedFramePoint3DReadOnly(DoubleSupplier scaleSupplier, FrameTuple3DReadOnly originalTuple)
   {
      return newLinkedFramePoint3DReadOnly(originalTuple, newLinkedPoint3DReadOnly(scaleSupplier, originalTuple));
   }

   /**
    * Creates a new frame vector that is linked to the {@code originalTuple} as follows:
    *
    * <pre>
    * linkedVector = scale * originalTuple
    * </pre>
    *
    * where the scale is obtained from the given {@code scaleSupplier}.
    *
    * @param scaleSupplier the supplier to get the scale.
    * @param originalTuple the reference tuple to scale. Not modified.
    * @return the new vector linked to {@code originalTuple}.
    */
   public static FrameVector3DReadOnly newLinkedFrameVector3DReadOnly(DoubleSupplier scaleSupplier, FrameTuple3DReadOnly originalTuple)
   {
      return newLinkedFrameVector3DReadOnly(originalTuple, newLinkedVector3DReadOnly(scaleSupplier, originalTuple));
   }

   /**
    * Creates a new point 2D that is a read-only view of the three coordinate suppliers expressed in
    * the reference frame provided by {@code referenceFrameHolder}.
    *
    * @param referenceFrameHolder the reference frame supplier. Not modified.
    * @param xSupplier            the x-coordinate supplier.
    * @param ySupplier            the y-coordinate supplier.
    * @return the new read-only frame point 2D.
    */
   public static FramePoint2DReadOnly newLinkedFramePoint2DReadOnly(ReferenceFrameHolder referenceFrameHolder, DoubleSupplier xSupplier,
                                                                    DoubleSupplier ySupplier)
   {
      return newLinkedFramePoint2DReadOnly(referenceFrameHolder, newLinkedPoint2DReadOnly(xSupplier, ySupplier));
   }

   /**
    * Creates a new vector 2D that is a read-only view of the three coordinate suppliers expressed in
    * the reference frame provided by {@code referenceFrameHolder}.
    *
    * @param referenceFrameHolder the reference frame supplier. Not modified.
    * @param xSupplier            the x-coordinate supplier.
    * @param ySupplier            the y-coordinate supplier.
    * @return the new read-only frame vector 2D.
    */
   public static FrameVector2DReadOnly newLinkedFrameVector2DReadOnly(ReferenceFrameHolder referenceFrameHolder, DoubleSupplier xSupplier,
                                                                      DoubleSupplier ySupplier)
   {
      return newLinkedFrameVector2DReadOnly(referenceFrameHolder, newLinkedVector2DReadOnly(xSupplier, ySupplier));
   }

   /**
    * Creates a new point 3D that is a read-only view of the three coordinate suppliers expressed in
    * the reference frame provided by {@code referenceFrameHolder}.
    *
    * @param referenceFrameHolder the reference frame supplier. Not modified.
    * @param xSupplier            the x-coordinate supplier.
    * @param ySupplier            the y-coordinate supplier.
    * @param zSupplier            the z-coordinate supplier.
    * @return the new read-only frame point 3D.
    */
   public static FramePoint3DReadOnly newLinkedFramePoint3DReadOnly(ReferenceFrameHolder referenceFrameHolder, DoubleSupplier xSupplier,
                                                                    DoubleSupplier ySupplier, DoubleSupplier zSupplier)
   {
      return newLinkedFramePoint3DReadOnly(referenceFrameHolder, newLinkedPoint3DReadOnly(xSupplier, ySupplier, zSupplier));
   }

   /**
    * Creates a new vector 3D that is a read-only view of the three coordinate suppliers expressed in
    * the reference frame provided by {@code referenceFrameHolder}.
    *
    * @param referenceFrameHolder the reference frame supplier. Not modified.
    * @param xSupplier            the x-coordinate supplier.
    * @param ySupplier            the y-coordinate supplier.
    * @param zSupplier            the z-coordinate supplier.
    * @return the new read-only frame vector 3D.
    */
   public static FrameVector3DReadOnly newLinkedFrameVector3DReadOnly(ReferenceFrameHolder referenceFrameHolder, DoubleSupplier xSupplier,
                                                                      DoubleSupplier ySupplier, DoubleSupplier zSupplier)
   {
      return newLinkedFrameVector3DReadOnly(referenceFrameHolder, newLinkedVector3DReadOnly(xSupplier, ySupplier, zSupplier));
   }

   /**
    * Creates a new point 2D that is a read-only view of the point expressed in the reference frame
    * provided by {@code referenceFrameHolder}.
    *
    * @param referenceFrameHolder the reference frame supplier. Not modified.
    * @param point                the point to link. Not modified.
    * @return the new read-only frame point 2D.
    */
   public static FramePoint2DReadOnly newLinkedFramePoint2DReadOnly(ReferenceFrameHolder referenceFrameHolder, Point2DReadOnly point)
   {
      return new FramePoint2DReadOnly()
      {
         @Override
         public double getX()
         {
            return point.getX();
         }

         @Override
         public double getY()
         {
            return point.getY();
         }

         @Override
         public ReferenceFrame getReferenceFrame()
         {
            return referenceFrameHolder.getReferenceFrame();
         }

         @Override
         public int hashCode()
         {
            return toIntHashCode(toIntHashCode(getX(), getY()), getReferenceFrame());
         }

         @Override
         public boolean equals(Object object)
         {
            if (object instanceof FramePoint2DReadOnly)
               return equals((FramePoint2DReadOnly) object);
            else
               return false;
         }

         @Override
         public String toString()
         {
            return EuclidFrameIOTools.getFrameTuple2DString(this);
         }
      };
   }

   /**
    * Creates a new vector 2D that is a read-only view of the vector expressed in the reference frame
    * provided by {@code referenceFrameHolder}.
    *
    * @param referenceFrameHolder the reference frame supplier. Not modified.
    * @param vector               the vector to link. Not modified.
    * @return the new read-only frame vector 2D.
    */
   public static FrameVector2DReadOnly newLinkedFrameVector2DReadOnly(ReferenceFrameHolder referenceFrameHolder, Vector2DReadOnly vector)
   {
      return new FrameVector2DReadOnly()
      {
         @Override
         public double getX()
         {
            return vector.getX();
         }

         @Override
         public double getY()
         {
            return vector.getY();
         }

         @Override
         public ReferenceFrame getReferenceFrame()
         {
            return referenceFrameHolder.getReferenceFrame();
         }

         @Override
         public int hashCode()
         {
            return toIntHashCode(toIntHashCode(getX(), getY()), getReferenceFrame());
         }

         @Override
         public boolean equals(Object object)
         {
            if (object instanceof FrameVector2DReadOnly)
               return equals((FrameVector2DReadOnly) object);
            else
               return false;
         }

         @Override
         public String toString()
         {
            return EuclidFrameIOTools.getFrameTuple2DString(this);
         }
      };
   }

   /**
    * Creates a new point 3D that is a read-only view of the point expressed in the reference frame
    * provided by {@code referenceFrameHolder}.
    *
    * @param referenceFrameHolder the reference frame supplier. Not modified.
    * @param point                the point to link. Not modified.
    * @return the new read-only frame point 3D.
    */
   public static FramePoint3DReadOnly newLinkedFramePoint3DReadOnly(ReferenceFrameHolder referenceFrameHolder, Point3DReadOnly point)
   {
      return new FramePoint3DReadOnly()
      {
         @Override
         public double getX()
         {
            return point.getX();
         }

         @Override
         public double getY()
         {
            return point.getY();
         }

         @Override
         public double getZ()
         {
            return point.getZ();
         }

         @Override
         public ReferenceFrame getReferenceFrame()
         {
            return referenceFrameHolder.getReferenceFrame();
         }

         @Override
         public int hashCode()
         {
            return toIntHashCode(toIntHashCode(getX(), getY(), getZ()), getReferenceFrame());
         }

         @Override
         public boolean equals(Object object)
         {
            if (object instanceof FramePoint3DReadOnly)
               return equals((FramePoint3DReadOnly) object);
            else
               return false;
         }

         @Override
         public String toString()
         {
            return EuclidFrameIOTools.getFrameTuple3DString(this);
         }
      };
   }

   /**
    * Creates a new vector 3D that is a read-only view of the vector expressed in the reference frame
    * provided by {@code referenceFrameHolder}.
    *
    * @param referenceFrameHolder the reference frame supplier. Not modified.
    * @param vector               the vector to link. Not modified.
    * @return the new read-only frame vector 3D.
    */
   public static FrameVector3DReadOnly newLinkedFrameVector3DReadOnly(ReferenceFrameHolder referenceFrameHolder, Vector3DReadOnly vector)
   {
      return new FrameVector3DReadOnly()
      {
         @Override
         public double getX()
         {
            return vector.getX();
         }

         @Override
         public double getY()
         {
            return vector.getY();
         }

         @Override
         public double getZ()
         {
            return vector.getZ();
         }

         @Override
         public ReferenceFrame getReferenceFrame()
         {
            return referenceFrameHolder.getReferenceFrame();
         }

         @Override
         public int hashCode()
         {
            return toIntHashCode(toIntHashCode(getX(), getY(), getZ()), getReferenceFrame());
         }

         @Override
         public boolean equals(Object object)
         {
            if (object instanceof FrameVector3DReadOnly)
               return equals((FrameVector3DReadOnly) object);
            else
               return false;
         }

         @Override
         public String toString()
         {
            return EuclidFrameIOTools.getFrameTuple3DString(this);
         }
      };
   }

   /**
    * Creates a new unit vector 2D that is a read-only view of the unit vector expressed in the
    * reference frame provided by {@code referenceFrameHolder}.
    *
    * @param referenceFrameHolder the reference frame supplier. Not modified.
    * @param unitVector           the vector to link. Not modified.
    * @return the new read-only frame unit vector 2D.
    */
   public static FrameUnitVector2DReadOnly newLinkedFrameUnitVector2DReadOnly(ReferenceFrameHolder referenceFrameHolder, UnitVector2DReadOnly unitVector)
   {
      return new FrameUnitVector2DReadOnly()
      {
         @Override
         public boolean isDirty()
         {
            return unitVector.isDirty();
         }

         @Override
         public ReferenceFrame getReferenceFrame()
         {
            return referenceFrameHolder.getReferenceFrame();
         }

         @Override
         public double getX()
         {
            return unitVector.getX();
         }

         @Override
         public double getY()
         {
            return unitVector.getY();
         }

         @Override
         public double getRawX()
         {
            return unitVector.getRawX();
         }

         @Override
         public double getRawY()
         {
            return unitVector.getRawY();
         }

         @Override
         public int hashCode()
         {
            return toIntHashCode(toIntHashCode(getX(), getY()), getReferenceFrame());
         }

         @Override
         public boolean equals(Object object)
         {
            if (object instanceof FrameVector2DReadOnly)
               return equals((FrameVector2DReadOnly) object);
            else
               return false;
         }

         @Override
         public String toString()
         {
            return EuclidFrameIOTools.getFrameTuple2DString(this);
         }
      };
   }

   /**
    * Creates a new unit vector 3D that is a read-only view of the unit vector expressed in the
    * reference frame provided by {@code referenceFrameHolder}.
    *
    * @param referenceFrameHolder the reference frame supplier. Not modified.
    * @param unitVector           the vector to link. Not modified.
    * @return the new read-only frame unit vector 3D.
    */
   public static FrameUnitVector3DReadOnly newLinkedFrameUnitVector3DReadOnly(ReferenceFrameHolder referenceFrameHolder, UnitVector3DReadOnly unitVector)
   {
      return new FrameUnitVector3DReadOnly()
      {
         @Override
         public boolean isDirty()
         {
            return unitVector.isDirty();
         }

         @Override
         public ReferenceFrame getReferenceFrame()
         {
            return referenceFrameHolder.getReferenceFrame();
         }

         @Override
         public double getX()
         {
            return unitVector.getX();
         }

         @Override
         public double getY()
         {
            return unitVector.getY();
         }

         @Override
         public double getZ()
         {
            return unitVector.getZ();
         }

         @Override
         public double getRawX()
         {
            return unitVector.getRawX();
         }

         @Override
         public double getRawY()
         {
            return unitVector.getRawY();
         }

         @Override
         public double getRawZ()
         {
            return unitVector.getRawZ();
         }

         @Override
         public int hashCode()
         {
            return toIntHashCode(toIntHashCode(getX(), getY(), getZ()), getReferenceFrame());
         }

         @Override
         public boolean equals(Object object)
         {
            if (object instanceof FrameVector3DReadOnly)
               return equals((FrameVector3DReadOnly) object);
            else
               return false;
         }

         @Override
         public String toString()
         {
            return EuclidFrameIOTools.getFrameTuple3DString(this);
         }
      };
   }

   /**
    * Creates a new unit rotation matrix that is a read-only view of the rotation matrix expressed in
    * the reference frame provided by {@code referenceFrameHolder}.
    *
    * @param referenceFrameHolder the reference frame supplier. Not modified.
    * @param rotationMatrix       the rotation matrix to link. Not modified.
    * @return the new read-only frame rotation matrix.
    */
   public static FrameRotationMatrixReadOnly newLinkedFrameRotationMatrixReadOnly(ReferenceFrameHolder referenceFrameHolder,
                                                                                  RotationMatrixReadOnly rotationMatrix)
   {
      return new FrameRotationMatrixReadOnly()
      {
         @Override
         public ReferenceFrame getReferenceFrame()
         {
            return referenceFrameHolder.getReferenceFrame();
         }

         @Override
         public boolean isDirty()
         {
            return rotationMatrix.isDirty();
         }

         @Override
         public double getM00()
         {
            return rotationMatrix.getM00();
         }

         @Override
         public double getM01()
         {
            return rotationMatrix.getM01();
         }

         @Override
         public double getM02()
         {
            return rotationMatrix.getM02();
         }

         @Override
         public double getM10()
         {
            return rotationMatrix.getM10();
         }

         @Override
         public double getM11()
         {
            return rotationMatrix.getM11();
         }

         @Override
         public double getM12()
         {
            return rotationMatrix.getM12();
         }

         @Override
         public double getM20()
         {
            return rotationMatrix.getM20();
         }

         @Override
         public double getM21()
         {
            return rotationMatrix.getM21();
         }

         @Override
         public double getM22()
         {
            return rotationMatrix.getM22();
         }

         @Override
         public int hashCode()
         {
            return toIntHashCode(toIntHashCode(getM00(), getM01(), getM02(), getM10(), getM11(), getM12(), getM20(), getM21(), getM22()), getReferenceFrame());
         }

         @Override
         public boolean equals(Object object)
         {
            if (object instanceof FrameMatrix3DReadOnly)
               return equals((FrameMatrix3DReadOnly) object);
            else
               return false;
         }

         @Override
         public String toString()
         {
            return EuclidFrameIOTools.getFrameMatrix3DString(this);
         }
      };
   }

   /**
    * Creates a new unit quaternion that is a read-only view of the quaternion expressed in the
    * reference frame provided by {@code referenceFrameHolder}.
    *
    * @param referenceFrameHolder the reference frame supplier. Not modified.
    * @param quaternion           the quaternion to link. Not modified.
    * @return the new read-only frame quaternion.
    */
   public static FrameQuaternionReadOnly newLinkedFrameQuaternionReadOnly(ReferenceFrameHolder referenceFrameHolder, QuaternionReadOnly quaternion)
   {
      return new FrameQuaternionReadOnly()
      {
         @Override
         public ReferenceFrame getReferenceFrame()
         {
            return referenceFrameHolder.getReferenceFrame();
         }

         @Override
         public double getX()
         {
            return quaternion.getX();
         }

         @Override
         public double getY()
         {
            return quaternion.getY();
         }

         @Override
         public double getZ()
         {
            return quaternion.getZ();
         }

         @Override
         public double getS()
         {
            return quaternion.getS();
         }

         @Override
         public int hashCode()
         {
            return toIntHashCode(toIntHashCode(getX(), getY(), getZ(), getS()), getReferenceFrame());
         }

         @Override
         public boolean equals(Object object)
         {
            if (object instanceof FrameQuaternionReadOnly)
               return equals((FrameQuaternionReadOnly) object);
            else
               return false;
         }

         @Override
         public String toString()
         {
            return EuclidFrameIOTools.getFrameTuple4DString(this);
         }
      };
   }

   /**
    * Creates a new frame point 2D that is a read-only view of the given {@code originalPoint} negated.
    *
    * @param originalPoint the original point to create linked negative point for. Not modified.
    * @return the negative read-only view of {@code originalPoint}.
    */
   public static FramePoint2DReadOnly newNegativeLinkedFramePoint2D(FramePoint2DReadOnly originalPoint)
   {
      return newLinkedFramePoint2DReadOnly(originalPoint, newNegativeLinkedPoint2D(originalPoint));
   }

   /**
    * Creates a new frame vector 2D that is a read-only view of the given {@code originalVector}
    * negated.
    *
    * @param originalVector the original vector to create linked negative vector for. Not modified.
    * @return the negative read-only view of {@code originalVector}.
    */
   public static FrameVector2DReadOnly newNegativeLinkedFrameVector2D(FrameVector2DReadOnly originalVector)
   {
      return newLinkedFrameVector2DReadOnly(originalVector, newNegativeLinkedVector2D(originalVector));
   }

   /**
    * Creates a new frame point 3D that is a read-only view of the given {@code originalPoint} negated.
    *
    * @param originalPoint the original point to create linked negative point for. Not modified.
    * @return the negative read-only view of {@code originalPoint}.
    */
   public static FramePoint3DReadOnly newNegativeLinkedFramePoint3D(FramePoint3DReadOnly originalPoint)
   {
      return newLinkedFramePoint3DReadOnly(originalPoint, newNegativeLinkedPoint3D(originalPoint));
   }

   /**
    * Creates a new frame vector 3D that is a read-only view of the given {@code originalVector}
    * negated.
    *
    * @param originalVector the original vector to create linked negative vector for. Not modified.
    * @return the negative read-only view of {@code originalVector}.
    */
   public static FrameVector3DReadOnly newNegativeLinkedFrameVector3D(FrameVector3DReadOnly originalVector)
   {
      return newLinkedFrameVector3DReadOnly(originalVector, newNegativeLinkedVector3D(originalVector));
   }

   /**
    * Creates a new frame point which reference frame is linked to the given
    * {@code referenceFrameHolder}.
    *
    * @param referenceFrameHolder the reference frame holder to link to the new frame point.
    * @return the new linked frame point.
    */
   public static FixedFramePoint2DBasics newFixedFramePoint2DBasics(ReferenceFrameHolder referenceFrameHolder)
   {
      return newLinkedFixedFramePoint2DBasics(referenceFrameHolder, new Point2D());
   }

   /**
    * Creates a new frame point which is linked to the given frameless point and
    * {@code referenceFrameHolder}.
    *
    * @param referenceFrameHolder the reference frame holder to link to the new frame point.
    * @param originalPoint        the point to link to the new frame point. Modifications on either the
    *                             {@code originalPoint} or the new frame point will be propagated to
    *                             the other.
    * @return the new linked frame point.
    */
   public static FixedFramePoint2DBasics newLinkedFixedFramePoint2DBasics(ReferenceFrameHolder referenceFrameHolder, Point2DBasics originalPoint)
   {
      return new FixedFramePoint2DBasics()
      {
         @Override
         public void setX(double x)
         {
            originalPoint.setX(x);
         }

         @Override
         public void setY(double y)
         {
            originalPoint.setY(y);
         }

         @Override
         public double getX()
         {
            return originalPoint.getX();
         }

         @Override
         public double getY()
         {
            return originalPoint.getY();
         }

         @Override
         public ReferenceFrame getReferenceFrame()
         {
            return referenceFrameHolder.getReferenceFrame();
         }

         @Override
         public boolean equals(Object object)
         {
            if (object == this)
               return true;
            else if (object instanceof FramePoint2DReadOnly)
               return equals((FramePoint2DReadOnly) object);
            else
               return false;
         }

         @Override
         public int hashCode()
         {
            return toIntHashCode(toIntHashCode(getX(), getY()), getReferenceFrame());
         }

         @Override
         public String toString()
         {
            return EuclidFrameIOTools.getFrameTuple2DString(this);
         }
      };
   }

   /**
    * Creates a new frame vector which reference frame is linked to the given
    * {@code referenceFrameHolder}.
    *
    * @param referenceFrameHolder the reference frame holder to link to the new frame vector.
    * @return the new linked frame vector.
    */
   public static FixedFrameVector2DBasics newFixedFrameVector2DBasics(ReferenceFrameHolder referenceFrameHolder)
   {
      return newLinkedFixedFrameVector2DBasics(referenceFrameHolder, new Vector2D());
   }

   /**
    * Creates a new frame vector which is linked to the given frameless vector and
    * {@code referenceFrameHolder}.
    *
    * @param referenceFrameHolder the reference frame holder to link to the new frame vector.
    * @param originalVector       the vector to link to the new frame vector. Modifications on either
    *                             the {@code originalVector} or the new frame vector will be propagated
    *                             to the other.
    * @return the new linked frame vector.
    */
   public static FixedFrameVector2DBasics newLinkedFixedFrameVector2DBasics(ReferenceFrameHolder referenceFrameHolder, Vector2DBasics originalVector)
   {
      return new FixedFrameVector2DBasics()
      {
         @Override
         public void setX(double x)
         {
            originalVector.setX(x);
         }

         @Override
         public void setY(double y)
         {
            originalVector.setY(y);
         }

         @Override
         public double getX()
         {
            return originalVector.getX();
         }

         @Override
         public double getY()
         {
            return originalVector.getY();
         }

         @Override
         public ReferenceFrame getReferenceFrame()
         {
            return referenceFrameHolder.getReferenceFrame();
         }

         @Override
         public boolean equals(Object object)
         {
            if (object == this)
               return true;
            else if (object instanceof FrameVector2DReadOnly)
               return equals((FrameVector2DReadOnly) object);
            else
               return false;
         }

         @Override
         public int hashCode()
         {
            return toIntHashCode(toIntHashCode(getX(), getY()), getReferenceFrame());
         }

         @Override
         public String toString()
         {
            return EuclidFrameIOTools.getFrameTuple2DString(this);
         }
      };
   }

   /**
    * Creates a new frame point which reference frame is linked to the given
    * {@code referenceFrameHolder}.
    *
    * @param referenceFrameHolder the reference frame holder to link to the new frame point.
    * @return the new linked frame point.
    */
   public static FixedFramePoint3DBasics newFixedFramePoint3DBasics(ReferenceFrameHolder referenceFrameHolder)
   {
      return newLinkedFixedFramePoint3DBasics(referenceFrameHolder, new Point3D());
   }

   /**
    * Creates a new frame point which is linked to the given frameless point and
    * {@code referenceFrameHolder}.
    *
    * @param referenceFrameHolder the reference frame holder to link to the new frame point.
    * @param originalPoint        the point to link to the new frame point. Modifications on either the
    *                             {@code originalPoint} or the new frame point will be propagated to
    *                             the other.
    * @return the new linked frame point.
    */
   public static FixedFramePoint3DBasics newLinkedFixedFramePoint3DBasics(ReferenceFrameHolder referenceFrameHolder, Point3DBasics originalPoint)
   {
      return new FixedFramePoint3DBasics()
      {
         @Override
         public void setX(double x)
         {
            originalPoint.setX(x);
         }

         @Override
         public void setY(double y)
         {
            originalPoint.setY(y);
         }

         @Override
         public void setZ(double z)
         {
            originalPoint.setZ(z);
         }

         @Override
         public double getX()
         {
            return originalPoint.getX();
         }

         @Override
         public double getY()
         {
            return originalPoint.getY();
         }

         @Override
         public double getZ()
         {
            return originalPoint.getZ();
         }

         @Override
         public ReferenceFrame getReferenceFrame()
         {
            return referenceFrameHolder.getReferenceFrame();
         }

         @Override
         public boolean equals(Object object)
         {
            if (object == this)
               return true;
            else if (object instanceof FramePoint3DReadOnly)
               return equals((FramePoint3DReadOnly) object);
            else
               return false;
         }

         @Override
         public int hashCode()
         {
            return toIntHashCode(toIntHashCode(getX(), getY(), getZ()), getReferenceFrame());
         }

         @Override
         public String toString()
         {
            return EuclidFrameIOTools.getFrameTuple3DString(this);
         }
      };
   }

   /**
    * Creates a new frame vector which reference frame is linked to the given
    * {@code referenceFrameHolder}.
    *
    * @param referenceFrameHolder the reference frame holder to link to the new frame vector.
    * @return the new linked frame vector.
    */
   public static FixedFrameVector3DBasics newFixedFrameVector3DBasics(ReferenceFrameHolder referenceFrameHolder)
   {
      return newLinkedFixedFrameVector3DBasics(referenceFrameHolder, new Vector3D());
   }

   /**
    * Creates a new frame vector which is linked to the given frameless vector and
    * {@code referenceFrameHolder}.
    *
    * @param referenceFrameHolder the reference frame holder to link to the new frame vector.
    * @param originalVector       the vector to link to the new frame vector. Modifications on either
    *                             the {@code originalVector} or the new frame vector will be propagated
    *                             to the other.
    * @return the new linked frame vector.
    */
   public static FixedFrameVector3DBasics newLinkedFixedFrameVector3DBasics(ReferenceFrameHolder referenceFrameHolder, Vector3DBasics originalVector)
   {
      return new FixedFrameVector3DBasics()
      {
         @Override
         public void setX(double x)
         {
            originalVector.setX(x);
         }

         @Override
         public void setY(double y)
         {
            originalVector.setY(y);
         }

         @Override
         public void setZ(double z)
         {
            originalVector.setZ(z);
         }

         @Override
         public double getX()
         {
            return originalVector.getX();
         }

         @Override
         public double getY()
         {
            return originalVector.getY();
         }

         @Override
         public double getZ()
         {
            return originalVector.getZ();
         }

         @Override
         public ReferenceFrame getReferenceFrame()
         {
            return referenceFrameHolder.getReferenceFrame();
         }

         @Override
         public boolean equals(Object object)
         {
            if (object == this)
               return true;
            else if (object instanceof FrameVector3DReadOnly)
               return equals((FrameVector3DReadOnly) object);
            else
               return false;
         }

         @Override
         public int hashCode()
         {
            return toIntHashCode(toIntHashCode(getX(), getY(), getZ()), getReferenceFrame());
         }

         @Override
         public String toString()
         {
            return EuclidFrameIOTools.getFrameTuple3DString(this);
         }
      };
   }

   /**
    * Creates a new vector 2D that is constrained to be a unit-length vector, i.e.
    * {@code vector.length() == 1.0}.
    * <p>
    * The new vector is initialized to {@link Axis2D#X}.
    * </p>
    *
    * @param referenceFrameHolder the reference frame holder to link to the new frame vector.
    * @return the new unitary vector.
    */
   public static FixedFrameUnitVector2DBasics newFixedFrameUnitVector2DBasics(ReferenceFrameHolder referenceFrameHolder)
   {
      return newFixedFrameUnitVector2DBasics(referenceFrameHolder, Axis2D.X);
   }

   /**
    * Creates a new vector 2D that is constrained to be a unit-length vector, i.e.
    * {@code vector.length() == 1.0}.
    *
    * @param referenceFrameHolder the reference frame holder to link to the new frame vector.
    * @param initialValue         the initial value for the new vector. Not modified.
    * @return the new unitary vector.
    */
   public static FixedFrameUnitVector2DBasics newFixedFrameUnitVector2DBasics(ReferenceFrameHolder referenceFrameHolder, Vector2DReadOnly initialValue)
   {
      return newFixedFrameUnitVector2DBasics(referenceFrameHolder, initialValue.getX(), initialValue.getY());
   }

   /**
    * Creates a new vector 2D that is constrained to be a unit-length vector, i.e.
    * {@code vector.length() == 1.0}.
    *
    * @param referenceFrameHolder the reference frame holder to link to the new frame vector.
    * @param initialX             the initial value for the x-component of the new vector.
    * @param initialY             the initial value for the y-component of the new vector.
    * @return the new unitary vector.
    */
   public static FixedFrameUnitVector2DBasics newFixedFrameUnitVector2DBasics(ReferenceFrameHolder referenceFrameHolder, double initialX, double initialY)
   {
      return newLinkedFixedFrameUnitVector2DBasics(referenceFrameHolder, new UnitVector2D(initialX, initialY));
   }

   /**
    * Creates a new frame unit vector which is linked to the given frameless unit vector and
    * {@code referenceFrameHolder}.
    *
    * @param referenceFrameHolder the reference frame holder to link to the new frame vector.
    * @param originalUnitVector   the unit vector to link to the new frame vector. Modifications on
    *                             either the {@code originalUnitVector} or the new frame vector will be
    *                             propagated to the other.
    * @return the new linked frame unit vector.
    */
   public static FixedFrameUnitVector2DBasics newLinkedFixedFrameUnitVector2DBasics(ReferenceFrameHolder referenceFrameHolder,
                                                                                    UnitVector2DBasics originalUnitVector)
   {
      return new FixedFrameUnitVector2DBasics()
      {
         @Override
         public void absolute()
         {
            originalUnitVector.absolute();
         }

         @Override
         public void negate()
         {
            originalUnitVector.negate();
         }

         @Override
         public void normalize()
         {
            originalUnitVector.normalize();
         }

         @Override
         public void markAsDirty()
         {
            originalUnitVector.markAsDirty();
         }

         @Override
         public boolean isDirty()
         {
            return originalUnitVector.isDirty();
         }

         @Override
         public void set(UnitVector2DReadOnly other)
         {
            originalUnitVector.set(other);
         }

         @Override
         public void setX(double x)
         {
            originalUnitVector.setX(x);
         }

         @Override
         public void setY(double y)
         {
            originalUnitVector.setY(y);
         }

         @Override
         public ReferenceFrame getReferenceFrame()
         {
            return referenceFrameHolder.getReferenceFrame();
         }

         @Override
         public double getX()
         {
            return originalUnitVector.getX();
         }

         @Override
         public double getY()
         {
            return originalUnitVector.getY();
         }

         @Override
         public double getRawX()
         {
            return originalUnitVector.getRawX();
         }

         @Override
         public double getRawY()
         {
            return originalUnitVector.getRawY();
         }

         @Override
         public int hashCode()
         {
            return toIntHashCode(toIntHashCode(getX(), getY()), getReferenceFrame());
         }

         @Override
         public boolean equals(Object object)
         {
            if (object instanceof FrameVector2DReadOnly)
               return equals((FrameVector2DReadOnly) object);
            else
               return false;
         }

         @Override
         public String toString()
         {
            return EuclidFrameIOTools.getFrameTuple2DString(this);
         }
      };
   }

   /**
    * Creates a new vector 3D that is constrained to be a unit-length vector, i.e.
    * {@code vector.length() == 1.0}.
    * <p>
    * The new vector is initialized to: {@link Axis3D#X}.
    * </p>
    *
    * @param referenceFrameHolder the reference frame holder to link to the new frame vector.
    * @return the new unitary vector.
    */
   public static FixedFrameUnitVector3DBasics newFixedFrameUnitVector3DBasics(ReferenceFrameHolder referenceFrameHolder)
   {
      return newFixedFrameUnitVector3DBasics(referenceFrameHolder, Axis3D.X);
   }

   /**
    * Creates a new vector 3D that is constrained to be a unit-length vector, i.e.
    * {@code vector.length() == 1.0}.
    *
    * @param referenceFrameHolder the reference frame holder to link to the new frame vector.
    * @param initialValue         the initial value for the new vector. Not modified.
    * @return the new unitary vector.
    */
   public static FixedFrameUnitVector3DBasics newFixedFrameUnitVector3DBasics(ReferenceFrameHolder referenceFrameHolder, Vector3DReadOnly initialValue)
   {
      return newFixedFrameUnitVector3DBasics(referenceFrameHolder, initialValue.getX(), initialValue.getY(), initialValue.getZ());
   }

   /**
    * Creates a new vector 3D that is constrained to be a unit-length vector, i.e.
    * {@code vector.length() == 1.0}.
    *
    * @param referenceFrameHolder the reference frame holder to link to the new frame vector.
    * @param initialX             the initial value for the x-component of the new vector.
    * @param initialY             the initial value for the y-component of the new vector.
    * @param initialZ             the initial value for the z-component of the new vector.
    * @return the new unitary vector.
    */
   public static FixedFrameUnitVector3DBasics newFixedFrameUnitVector3DBasics(ReferenceFrameHolder referenceFrameHolder, double initialX, double initialY,
                                                                              double initialZ)
   {
      return newLinkedFixedFrameUnitVector3DBasics(referenceFrameHolder, new UnitVector3D(initialX, initialY, initialZ));
   };

   /**
    * Creates a new frame unit vector which is linked to the given frameless unit vector and
    * {@code referenceFrameHolder}.
    *
    * @param referenceFrameHolder the reference frame holder to link to the new frame vector.
    * @param originalUnitVector   the unit vector to link to the new frame vector. Modifications on
    *                             either the {@code originalUnitVector} or the new frame vector will be
    *                             propagated to the other.
    * @return the new linked frame unit vector.
    */
   public static FixedFrameUnitVector3DBasics newLinkedFixedFrameUnitVector3DBasics(ReferenceFrameHolder referenceFrameHolder,
                                                                                    UnitVector3DBasics originalUnitVector)
   {
      return new FixedFrameUnitVector3DBasics()
      {
         @Override
         public void absolute()
         {
            originalUnitVector.absolute();
         }

         @Override
         public void negate()
         {
            originalUnitVector.negate();
         }

         @Override
         public void normalize()
         {
            originalUnitVector.normalize();
         }

         @Override
         public void markAsDirty()
         {
            originalUnitVector.markAsDirty();
         }

         @Override
         public boolean isDirty()
         {
            return originalUnitVector.isDirty();
         }

         @Override
         public void set(UnitVector3DReadOnly other)
         {
            originalUnitVector.set(other);
         }

         @Override
         public void setX(double x)
         {
            originalUnitVector.setX(x);
         }

         @Override
         public void setY(double y)
         {
            originalUnitVector.setY(y);
         }

         @Override
         public void setZ(double z)
         {
            originalUnitVector.setZ(z);
         }

         @Override
         public ReferenceFrame getReferenceFrame()
         {
            return referenceFrameHolder.getReferenceFrame();
         }

         @Override
         public double getX()
         {
            return originalUnitVector.getX();
         }

         @Override
         public double getY()
         {
            return originalUnitVector.getY();
         }

         @Override
         public double getZ()
         {
            return originalUnitVector.getZ();
         }

         @Override
         public double getRawX()
         {
            return originalUnitVector.getRawX();
         }

         @Override
         public double getRawY()
         {
            return originalUnitVector.getRawY();
         }

         @Override
         public double getRawZ()
         {
            return originalUnitVector.getRawZ();
         }

         @Override
         public int hashCode()
         {
            return toIntHashCode(toIntHashCode(getX(), getY(), getZ()), getReferenceFrame());
         }

         @Override
         public boolean equals(Object object)
         {
            if (object instanceof FrameVector3DReadOnly)
               return equals((FrameVector3DReadOnly) object);
            else
               return false;
         }

         @Override
         public String toString()
         {
            return EuclidFrameIOTools.getFrameTuple3DString(this);
         }
      };
   }

   /**
    * Creates a new frame orientation which reference frame is linked to the given
    * {@code referenceFrameHolder}.
    *
    * @param referenceFrameHolder the reference frame holder to link to the new frame orientation.
    * @return the new linked frame orientation.
    */
   public static FixedFrameOrientation2DBasics newFixedFrameOrientation2DBasics(ReferenceFrameHolder referenceFrameHolder)
   {
      return newLinkedFixedFrameOrientation2DBasics(referenceFrameHolder, new Orientation2D());
   }

   /**
    * Creates a new frame orientation which reference frame is linked to the given frameless
    * orientation and {@code referenceFrameHolder}.
    *
    * @param referenceFrameHolder the reference frame holder to link to the new frame orientation.
    * @param originalOrientation  the orientation to link to the new frame orientation. Modifications
    *                             on either the {@code originalOriginal} or the new frame vector will
    *                             be propagated to the other.
    * @return the new linked frame vector.
    */
   public static FixedFrameOrientation2DBasics newLinkedFixedFrameOrientation2DBasics(ReferenceFrameHolder referenceFrameHolder,
                                                                                      Orientation2DBasics originalOrientation)
   {
      return new FixedFrameOrientation2DBasics()
      {
         @Override
         public void setYaw(double yaw)
         {
            originalOrientation.setYaw(yaw);
         }

         @Override
         public ReferenceFrame getReferenceFrame()
         {
            return referenceFrameHolder.getReferenceFrame();
         }

         @Override
         public double getYaw()
         {
            return originalOrientation.getYaw();
         }

         @Override
         public void applyTransform(Transform transform)
         {
            originalOrientation.applyTransform(transform);
         }

         @Override
         public void applyInverseTransform(Transform transform)
         {
            originalOrientation.applyInverseTransform(transform);
         }

         @Override
         public boolean equals(Object object)
         {
            if (object == this)
               return true;
            else if (object instanceof FrameOrientation2DReadOnly)
               return equals((FrameOrientation2DReadOnly) object);
            else
               return false;
         }

         @Override
         public int hashCode()
         {
            return toIntHashCode(toIntHashCode(getYaw()), getReferenceFrame());
         }

         @Override
         public String toString()
         {
            return EuclidFrameIOTools.getFrameOrientation2DString(this);
         }
      };
   }

   /**
    * Creates a new frame quaternion which reference frame is linked to the given
    * {@code referenceFrameHolder}.
    *
    * @param referenceFrameHolder the reference frame holder to link to the new frame quaternion.
    * @return the new linked frame quaternion.
    */
   public static FixedFrameQuaternionBasics newFixedFrameQuaternionBasics(ReferenceFrameHolder referenceFrameHolder)
   {
      return newLinkedFixedFrameQuaternionBasics(referenceFrameHolder, new Quaternion());
   }

   /**
    * Creates a new frame quaternion which reference frame is linked to the given frameless quaternion
    * and {@code referenceFrameHolder}.
    *
    * @param referenceFrameHolder the reference frame holder to link to the new frame quaternion.
    * @param originalQuaternion   the quaternion to link to the new frame quaternion. Modifications on
    *                             either the {@code originalQuaternion} or the new frame quaternion
    *                             will be propagated to the other.
    * @return the new linked frame quaternion.
    */
   public static FixedFrameQuaternionBasics newLinkedFixedFrameQuaternionBasics(ReferenceFrameHolder referenceFrameHolder, QuaternionBasics originalQuaternion)
   {
      return new FixedFrameQuaternionBasics()
      {
         @Override
         public void setUnsafe(double qx, double qy, double qz, double qs)
         {
            originalQuaternion.setUnsafe(qx, qy, qz, qs);
         }

         @Override
         public ReferenceFrame getReferenceFrame()
         {
            return referenceFrameHolder.getReferenceFrame();
         }

         @Override
         public double getX()
         {
            return originalQuaternion.getX();
         }

         @Override
         public double getY()
         {
            return originalQuaternion.getY();
         }

         @Override
         public double getZ()
         {
            return originalQuaternion.getZ();
         }

         @Override
         public double getS()
         {
            return originalQuaternion.getS();
         }

         @Override
         public boolean equals(Object object)
         {
            if (object == this)
               return true;
            else if (object instanceof FrameQuaternionReadOnly)
               return equals((FrameQuaternionReadOnly) object);
            else
               return false;
         }

         @Override
         public int hashCode()
         {
            return toIntHashCode(toIntHashCode(getX(), getY(), getZ(), getS()), getReferenceFrame());
         }

         @Override
         public String toString()
         {
            return EuclidFrameIOTools.getFrameTuple4DString(this);
         }
      };
   }

   /**
    * Creates a new frame rotation matrix which reference frame is linked to the given
    * {@code referenceFrameHolder}.
    *
    * @param referenceFrameHolder the reference frame holder to link to the new frame rotation matrix.
    * @return the new linked frame rotation matrix.
    */
   public static FixedFrameRotationMatrixBasics newFixedFrameRotationMatrixBasics(ReferenceFrameHolder referenceFrameHolder)
   {
      return newLinkedFixedFrameRotationMatrixBasics(referenceFrameHolder, new RotationMatrix());
   }

   /**
    * Creates a new frame rotation matrix which reference frame is linked to the given frameless
    * rotation matrix and {@code referenceFrameHolder}.
    *
    * @param referenceFrameHolder   the reference frame holder to link to the new frame rotation
    *                               matrix.
    * @param originalRotationMatrix the rotation matrix to link to the new frame rotation matrix.
    *                               Modifications on either the {@code originalRotationMatrix} or the
    *                               new frame rotation matrix will be propagated to the other.
    * @return the new linked frame rotation matrix.
    */
   public static FixedFrameRotationMatrixBasics newLinkedFixedFrameRotationMatrixBasics(ReferenceFrameHolder referenceFrameHolder,
                                                                                        RotationMatrixBasics originalRotationMatrix)
   {
      return new FixedFrameRotationMatrixBasics()
      {
         @Override
         public void setUnsafe(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22)
         {
            originalRotationMatrix.setUnsafe(m00, m01, m02, m10, m11, m12, m20, m21, m22);
         }

         @Override
         public void set(RotationMatrixReadOnly other)
         {
            originalRotationMatrix.set(other);
         }

         @Override
         public void setIdentity()
         {
            originalRotationMatrix.setIdentity();
         }

         @Override
         public void setToNaN()
         {
            originalRotationMatrix.setToNaN();
         }

         @Override
         public void normalize()
         {
            originalRotationMatrix.normalize();
         }

         @Override
         public boolean isIdentity()
         {
            return originalRotationMatrix.isIdentity();
         }

         @Override
         public void transpose()
         {
            originalRotationMatrix.transpose();
         }

         @Override
         public ReferenceFrame getReferenceFrame()
         {
            return referenceFrameHolder.getReferenceFrame();
         }

         @Override
         public boolean isDirty()
         {
            return originalRotationMatrix.isDirty();
         }

         /** {@inheritDoc} */
         @Override
         public double getM00()
         {
            return originalRotationMatrix.getM00();
         }

         /** {@inheritDoc} */
         @Override
         public double getM01()
         {
            return originalRotationMatrix.getM01();
         }

         /** {@inheritDoc} */
         @Override
         public double getM02()
         {
            return originalRotationMatrix.getM02();
         }

         /** {@inheritDoc} */
         @Override
         public double getM10()
         {
            return originalRotationMatrix.getM10();
         }

         /** {@inheritDoc} */
         @Override
         public double getM11()
         {
            return originalRotationMatrix.getM11();
         }

         /** {@inheritDoc} */
         @Override
         public double getM12()
         {
            return originalRotationMatrix.getM12();
         }

         /** {@inheritDoc} */
         @Override
         public double getM20()
         {
            return originalRotationMatrix.getM20();
         }

         /** {@inheritDoc} */
         @Override
         public double getM21()
         {
            return originalRotationMatrix.getM21();
         }

         /** {@inheritDoc} */
         @Override
         public double getM22()
         {
            return originalRotationMatrix.getM22();
         }

         @Override
         public boolean equals(Object object)
         {
            if (object == this)
               return true;
            else if (object instanceof FrameRotationMatrixReadOnly)
               return equals((FrameRotationMatrixReadOnly) object);
            else
               return false;
         }

         @Override
         public int hashCode()
         {
            return toIntHashCode(toIntHashCode(getM00(), getM01(), getM02(), getM10(), getM11(), getM12(), getM20(), getM21(), getM22()), getReferenceFrame());
         }

         @Override
         public String toString()
         {
            return EuclidFrameIOTools.getFrameMatrix3DString(this);
         }
      };
   }

   /**
    * Creates a new frame bounding box which reference frame is linked to the given
    * {@code referenceFrameHolder}.
    *
    * @param referenceFrameHolder the reference frame holder to link to the new frame bounding box.
    * @return the new linked frame bounding box.
    */
   public static FixedFrameBoundingBox2DBasics newFixedFrameBoundingBox2DBasics(ReferenceFrameHolder referenceFrameHolder)
   {
      return newLinkedFixedFrameBoundingBox2DBasics(referenceFrameHolder, new BoundingBox2D());
   }

   /**
    * Creates a new frame bounding box which reference frame is linked to the given frameless bounding
    * box and {@code referenceFrameHolder}.
    *
    * @param referenceFrameHolder the reference frame holder to link to the new frame bounding box.
    * @param originalBoundingBox  the bounding box to link to the new frame bounding box. Modifications
    *                             on either the {@code originalBoundingBox} or the new frame bounding
    *                             box will be propagated to the other.
    * @return the new linked frame bounding box.
    */
   public static FixedFrameBoundingBox2DBasics newLinkedFixedFrameBoundingBox2DBasics(ReferenceFrameHolder referenceFrameHolder,
                                                                                      BoundingBox2DBasics originalBoundingBox)
   {
      FixedFrameBoundingBox2DBasics fixedFrameBoundingBox2DBasics = new FixedFrameBoundingBox2DBasics()
      {
         private final FixedFramePoint2DBasics minPoint = newLinkedFixedFramePoint2DBasics(referenceFrameHolder, originalBoundingBox.getMinPoint());
         private final FixedFramePoint2DBasics maxPoint = newLinkedFixedFramePoint2DBasics(referenceFrameHolder, originalBoundingBox.getMaxPoint());

         @Override
         public ReferenceFrame getReferenceFrame()
         {
            return referenceFrameHolder.getReferenceFrame();
         }

         @Override
         public FixedFramePoint2DBasics getMinPoint()
         {
            return minPoint;
         }

         @Override
         public FixedFramePoint2DBasics getMaxPoint()
         {
            return maxPoint;
         }

         @Override
         public boolean equals(Object object)
         {
            if (object instanceof FrameBoundingBox2DReadOnly)
               return equals((FrameBoundingBox2DReadOnly) object);
            else
               return false;
         }

         @Override
         public int hashCode()
         {
            return toIntHashCode(minPoint, maxPoint);
         }

         @Override
         public String toString()
         {
            return EuclidFrameIOTools.getFrameBoundingBox2DString(this);
         }
      };

      return fixedFrameBoundingBox2DBasics;
   }

   /**
    * Creates a new frame bounding box which reference frame is linked to the given
    * {@code referenceFrameHolder}.
    *
    * @param referenceFrameHolder the reference frame holder to link to the new frame bounding box.
    * @return the new linked frame bounding box.
    */
   public static FixedFrameBoundingBox3DBasics newFixedFrameBoundingBox3DBasics(ReferenceFrameHolder referenceFrameHolder)
   {
      return newLinkedFixedFrameBoundingBox3DBasics(referenceFrameHolder, new BoundingBox3D());
   }

   /**
    * Creates a new frame bounding box which reference frame is linked to the given frameless bounding
    * box and {@code referenceFrameHolder}.
    *
    * @param referenceFrameHolder the reference frame holder to link to the new frame bounding box.
    * @param originalBoundingBox  the bounding box to link to the new frame bounding box. Modifications
    *                             on either the {@code originalBoundingBox} or the new frame bounding
    *                             box will be propagated to the other.
    * @return the new linked frame bounding box.
    */
   public static FixedFrameBoundingBox3DBasics newLinkedFixedFrameBoundingBox3DBasics(ReferenceFrameHolder referenceFrameHolder,
                                                                                      BoundingBox3DBasics originalBoundingBox)
   {
      FixedFrameBoundingBox3DBasics fixedFrameBoundingBox3DBasics = new FixedFrameBoundingBox3DBasics()
      {
         private final FixedFramePoint3DBasics minPoint = newLinkedFixedFramePoint3DBasics(referenceFrameHolder, originalBoundingBox.getMinPoint());
         private final FixedFramePoint3DBasics maxPoint = newLinkedFixedFramePoint3DBasics(referenceFrameHolder, originalBoundingBox.getMaxPoint());

         @Override
         public ReferenceFrame getReferenceFrame()
         {
            return referenceFrameHolder.getReferenceFrame();
         }

         @Override
         public FixedFramePoint3DBasics getMinPoint()
         {
            return minPoint;
         }

         @Override
         public FixedFramePoint3DBasics getMaxPoint()
         {
            return maxPoint;
         }

         @Override
         public boolean equals(Object object)
         {
            if (object instanceof FrameBoundingBox3DReadOnly)
               return equals((FrameBoundingBox3DReadOnly) object);
            else
               return false;
         }

         @Override
         public int hashCode()
         {
            return toIntHashCode(minPoint, maxPoint);
         }

         @Override
         public String toString()
         {
            return EuclidFrameIOTools.getFrameBoundingBox3DString(this);
         }
      };

      return fixedFrameBoundingBox3DBasics;
   }

   /**
    * Creates a linked frame point that can be used to observe access to the source point's
    * coordinates.
    * 
    * @param valueAccessedListener the listener to be notified whenever a coordinate of the point is
    *                              being accessed. The corresponding constant {@link Axis2D} will be
    *                              passed to indicate the coordinate being accessed.
    * @param source                the original frame point to link and observe. Not modified.
    * @return the observable frame point.
    */
   public static FramePoint2DReadOnly newObservableFramePoint2DReadOnly(Consumer<Axis2D> valueAccessedListener, FramePoint2DReadOnly source)
   {
      return newLinkedFramePoint2DReadOnly(source, newObservablePoint2DReadOnly(valueAccessedListener, source));
   }

   /**
    * Creates a linked frame point that can be used to observe access to the source point's
    * coordinates.
    * 
    * @param valueAccessedListener the listener to be notified whenever a coordinate of the point is
    *                              being accessed. The corresponding constant {@link Axis3D} will be
    *                              passed to indicate the coordinate being accessed.
    * @param source                the original frame point to link and observe. Not modified.
    * @return the observable frame point.
    */
   public static FramePoint3DReadOnly newObservableFramePoint3DReadOnly(Consumer<Axis3D> valueAccessedListener, FramePoint3DReadOnly source)
   {
      return newLinkedFramePoint3DReadOnly(source, newObservablePoint3DReadOnly(valueAccessedListener, source));
   }

   /**
    * Creates a linked frame vector that can be used to observe access to the source vector's
    * coordinates.
    * 
    * @param valueAccessedListener the listener to be notified whenever a coordinate of the vector is
    *                              being accessed. The corresponding constant {@link Axis2D} will be
    *                              passed to indicate the coordinate being accessed.
    * @param source                the original frame vector to link and observe. Not modified.
    * @return the observable frame vector.
    */
   public static FrameVector2DReadOnly newObservableFrameVector2DReadOnly(Consumer<Axis2D> valueAccessedListener, FrameVector2DReadOnly source)
   {
      return newLinkedFrameVector2DReadOnly(source, newObservableVector2DReadOnly(valueAccessedListener, source));
   }

   /**
    * Creates a linked frame vector that can be used to observe access to the source vector's
    * coordinates.
    * 
    * @param valueAccessedListener the listener to be notified whenever a coordinate of the vector is
    *                              being accessed. The corresponding constant {@link Axis3D} will be
    *                              passed to indicate the coordinate being accessed.
    * @param source                the original frame vector to link and observe. Not modified.
    * @return the observable frame vector.
    */
   public static FrameVector3DReadOnly newObservableFrameVector3DReadOnly(Consumer<Axis3D> valueAccessedListener, FrameVector3DReadOnly source)
   {
      return newLinkedFrameVector3DReadOnly(source, newObservableVector3DReadOnly(valueAccessedListener, source));
   }

   /**
    * Creates a linked frame unit vector that can be used to observe access to the source unit vector's
    * coordinates.
    * 
    * @param valueAccessedListener the listener to be notified whenever a coordinate of the unit vector
    *                              is being accessed. The corresponding constant {@link Axis2D} will be
    *                              passed to indicate the coordinate being accessed.
    * @param source                the original frame unit vector to link and observe. Not modified.
    * @return the observable frame unit vector.
    */
   public static FrameUnitVector2DReadOnly newObservableFrameUnitVector2DReadOnly(Consumer<Axis2D> valueAccessedListener, FrameUnitVector2DReadOnly source)
   {
      return newLinkedFrameUnitVector2DReadOnly(source, newObservableUnitVector2DReadOnly(valueAccessedListener, source));
   }

   /**
    * Creates a linked frame unit vector that can be used to observe access to the source unit vector's
    * coordinates.
    * 
    * @param valueAccessedListener the listener to be notified whenever a coordinate of the unit vector
    *                              is being accessed. The corresponding constant {@link Axis3D} will be
    *                              passed to indicate the coordinate being accessed.
    * @param source                the original frame unit vector to link and observe. Not modified.
    * @return the observable frame unit vector.
    */
   public static FrameUnitVector3DReadOnly newObservableFrameUnitVector3DReadOnly(Consumer<Axis3D> valueAccessedListener, FrameUnitVector3DReadOnly source)
   {
      return newLinkedFrameUnitVector3DReadOnly(source, newObservableUnitVector3DReadOnly(valueAccessedListener, source));
   }

   /**
    * Creates a linked frame rotation matrix that can be used to observe access to the source rotation
    * matrix's coordinates.
    * 
    * @param valueAccessedListener the listener to be notified whenever a component of the rotation
    *                              matrix is being accessed. The corresponding constants {@link Axis3D}
    *                              will be passed to indicate the row and column respectively of the
    *                              coefficient being accessed.
    * @param source                the original frame rotation matrix to link and observe. Not
    *                              modified.
    * @return the observable frame rotation matrix.
    */
   public static FrameRotationMatrixReadOnly newObservableFrameRotationMatrixReadOnly(BiConsumer<Axis3D, Axis3D> valueAccessedListener,
                                                                                      FrameRotationMatrixReadOnly source)
   {
      return newLinkedFrameRotationMatrixReadOnly(source, newObservableRotationMatrixReadOnly(valueAccessedListener, source));
   }

   /**
    * Creates a linked frame quaternion that can be used to observe access to the source quaternion's
    * coordinates.
    * 
    * @param valueAccessedListener the listener to be notified whenever a component of the quaternion
    *                              is being accessed. The index of the component being accessed will be
    *                              passed.
    * @param source                the original frame quaternion to link and observe. Not modified.
    * @return the observable frame quaternion.
    */
   public static FrameQuaternionReadOnly newObservableFrameQuaternionReadOnly(IntConsumer valueAccessedListener, FrameQuaternionReadOnly source)
   {
      return newLinkedFrameQuaternionReadOnly(source, newObservableQuaternionReadOnly(valueAccessedListener, source));
   }

   /**
    * Creates a new frame point that can be used to observe read and write operations.
    * 
    * @param referenceFrameHolder  the reference frame supplier for the new frame point. Not modified.
    * @param valueChangedListener  the listener to be notified whenever a coordinate of the point has
    *                              been modified. The corresponding constant {@link Axis2D} will be
    *                              passed to indicate the coordinate that was changed alongside its new
    *                              value. Can be {@code null}.
    * @param valueAccessedListener the listener to be notified whenever a coordinate of the point is
    *                              being accessed. The corresponding constant {@link Axis2D} will be
    *                              passed to indicate the coordinate being accessed. Can be
    *                              {@code null}.
    * @return the observable frame point.
    */
   public static FixedFramePoint2DBasics newObservableFixedFramePoint2DBasics(ReferenceFrameHolder referenceFrameHolder,
                                                                              ObjDoubleConsumer<Axis2D> valueChangedListener,
                                                                              Consumer<Axis2D> valueAccessedListener)
   {
      return newObservableFixedFramePoint2DBasics(valueChangedListener, valueAccessedListener, newFixedFramePoint2DBasics(referenceFrameHolder));
   }

   /**
    * Creates a linked frame point that can be used to observe read and write operations on the source.
    * 
    * @param valueChangedListener  the listener to be notified whenever a coordinate of the point has
    *                              been modified. The corresponding constant {@link Axis2D} will be
    *                              passed to indicate the coordinate that was changed alongside its new
    *                              value. Can be {@code null}.
    * @param valueAccessedListener the listener to be notified whenever a coordinate of the point is
    *                              being accessed. The corresponding constant {@link Axis2D} will be
    *                              passed to indicate the coordinate being accessed. Can be
    *                              {@code null}.
    * @param source                the original point to link and observe. Modifiable via the linked
    *                              point interface.
    * @return the observable frame point.
    */
   public static FixedFramePoint2DBasics newObservableFixedFramePoint2DBasics(ObjDoubleConsumer<Axis2D> valueChangedListener,
                                                                              Consumer<Axis2D> valueAccessedListener, FixedFramePoint2DBasics source)
   {
      return newLinkedFixedFramePoint2DBasics(source, newObservablePoint2DBasics(valueChangedListener, valueAccessedListener, source));
   }

   /**
    * Creates a new frame point that can be used to observe read and write operations.
    * 
    * @param referenceFrameHolder  the reference frame supplier for the new frame point. Not modified.
    * @param valueChangedListener  the listener to be notified whenever a coordinate of the point has
    *                              been modified. The corresponding constant {@link Axis3D} will be
    *                              passed to indicate the coordinate that was changed alongside its new
    *                              value. Can be {@code null}.
    * @param valueAccessedListener the listener to be notified whenever a coordinate of the point is
    *                              being accessed. The corresponding constant {@link Axis3D} will be
    *                              passed to indicate the coordinate being accessed. Can be
    *                              {@code null}.
    * @return the observable frame point.
    */
   public static FixedFramePoint3DBasics newObservableFixedFramePoint3DBasics(ReferenceFrameHolder referenceFrameHolder,
                                                                              ObjDoubleConsumer<Axis3D> valueChangedListener,
                                                                              Consumer<Axis3D> valueAccessedListener)
   {
      return newObservableFixedFramePoint3DBasics(valueChangedListener, valueAccessedListener, newFixedFramePoint3DBasics(referenceFrameHolder));
   }

   /**
    * Creates a linked frame point that can be used to observe read and write operations on the source.
    * 
    * @param valueChangedListener  the listener to be notified whenever a coordinate of the point has
    *                              been modified. The corresponding constant {@link Axis3D} will be
    *                              passed to indicate the coordinate that was changed alongside its new
    *                              value. Can be {@code null}.
    * @param valueAccessedListener the listener to be notified whenever a coordinate of the point is
    *                              being accessed. The corresponding constant {@link Axis3D} will be
    *                              passed to indicate the coordinate being accessed. Can be
    *                              {@code null}.
    * @param source                the original point to link and observe. Modifiable via the linked
    *                              point interface.
    * @return the observable frame point.
    */
   public static FixedFramePoint3DBasics newObservableFixedFramePoint3DBasics(ObjDoubleConsumer<Axis3D> valueChangedListener,
                                                                              Consumer<Axis3D> valueAccessedListener, FixedFramePoint3DBasics source)
   {
      return newLinkedFixedFramePoint3DBasics(source, newObservablePoint3DBasics(valueChangedListener, valueAccessedListener, source));
   }

   /**
    * Creates a new frame vector that can be used to observe read and write operations.
    * 
    * @param referenceFrameHolder  the reference frame supplier for the new frame vector. Not modified.
    * @param valueChangedListener  the listener to be notified whenever a coordinate of the vector has
    *                              been modified. The corresponding constant {@link Axis2D} will be
    *                              passed to indicate the coordinate that was changed alongside its new
    *                              value. Can be {@code null}.
    * @param valueAccessedListener the listener to be notified whenever a coordinate of the vector is
    *                              being accessed. The corresponding constant {@link Axis2D} will be
    *                              passed to indicate the coordinate being accessed. Can be
    *                              {@code null}.
    * @return the observable frame vector.
    */
   public static FixedFrameVector2DBasics newObservableFixedFrameVector2DBasics(ReferenceFrameHolder referenceFrameHolder,
                                                                                ObjDoubleConsumer<Axis2D> valueChangedListener,
                                                                                Consumer<Axis2D> valueAccessedListener)
   {
      return newObservableFixedFrameVector2DBasics(valueChangedListener, valueAccessedListener, newFixedFrameVector2DBasics(referenceFrameHolder));
   }

   /**
    * Creates a linked frame vector that can be used to observe read and write operations on the
    * source.
    * 
    * @param valueChangedListener  the listener to be notified whenever a coordinate of the vector has
    *                              been modified. The corresponding constant {@link Axis2D} will be
    *                              passed to indicate the coordinate that was changed alongside its new
    *                              value. Can be {@code null}.
    * @param valueAccessedListener the listener to be notified whenever a coordinate of the vector is
    *                              being accessed. The corresponding constant {@link Axis2D} will be
    *                              passed to indicate the coordinate being accessed. Can be
    *                              {@code null}.
    * @param source                the original vector to link and observe. Modifiable via the linked
    *                              vector interface.
    * @return the observable frame vector.
    */
   public static FixedFrameVector2DBasics newObservableFixedFrameVector2DBasics(ObjDoubleConsumer<Axis2D> valueChangedListener,
                                                                                Consumer<Axis2D> valueAccessedListener, FixedFrameVector2DBasics source)
   {
      return newLinkedFixedFrameVector2DBasics(source, newObservableVector2DBasics(valueChangedListener, valueAccessedListener, source));
   }

   /**
    * Creates a new frame vector that can be used to observe read and write operations.
    * 
    * @param referenceFrameHolder  the reference frame supplier for the new frame vector. Not modified.
    * @param valueChangedListener  the listener to be notified whenever a coordinate of the vector has
    *                              been modified. The corresponding constant {@link Axis3D} will be
    *                              passed to indicate the coordinate that was changed alongside its new
    *                              value. Can be {@code null}.
    * @param valueAccessedListener the listener to be notified whenever a coordinate of the vector is
    *                              being accessed. The corresponding constant {@link Axis3D} will be
    *                              passed to indicate the coordinate being accessed. Can be
    *                              {@code null}.
    * @return the observable frame vector.
    */
   public static FixedFrameVector3DBasics newObservableFixedFrameVector3DBasics(ReferenceFrameHolder referenceFrameHolder,
                                                                                ObjDoubleConsumer<Axis3D> valueChangedListener,
                                                                                Consumer<Axis3D> valueAccessedListener)
   {
      return newObservableFixedFrameVector3DBasics(valueChangedListener, valueAccessedListener, newFixedFrameVector3DBasics(referenceFrameHolder));
   }

   /**
    * Creates a linked frame vector that can be used to observe read and write operations on the
    * source.
    * 
    * @param valueChangedListener  the listener to be notified whenever a coordinate of the vector has
    *                              been modified. The corresponding constant {@link Axis3D} will be
    *                              passed to indicate the coordinate that was changed alongside its new
    *                              value. Can be {@code null}.
    * @param valueAccessedListener the listener to be notified whenever a coordinate of the vector is
    *                              being accessed. The corresponding constant {@link Axis3D} will be
    *                              passed to indicate the coordinate being accessed. Can be
    *                              {@code null}.
    * @param source                the original vector to link and observe. Modifiable via the linked
    *                              vector interface.
    * @return the observable frame vector.
    */
   public static FixedFrameVector3DBasics newObservableFixedFrameVector3DBasics(ObjDoubleConsumer<Axis3D> valueChangedListener,
                                                                                Consumer<Axis3D> valueAccessedListener, FixedFrameVector3DBasics source)
   {
      return newLinkedFixedFrameVector3DBasics(source, newObservableVector3DBasics(valueChangedListener, valueAccessedListener, source));
   }

   /**
    * Creates a new frame unit vector that can be used to observe read and write operations.
    * 
    * @param referenceFrameHolder  the reference frame supplier for the new frame unit vector. Not
    *                              modified.
    * @param valueChangedListener  the listener to be notified whenever a coordinate of the unit vector
    *                              has been modified. The corresponding constant {@link Axis2D} will be
    *                              passed to indicate the coordinate that was changed alongside its new
    *                              value. Can be {@code null}.
    * @param valueAccessedListener the listener to be notified whenever a coordinate of the unit vector
    *                              is being accessed. The corresponding constant {@link Axis2D} will be
    *                              passed to indicate the coordinate being accessed. Can be
    *                              {@code null}.
    * @return the observable frame unit vector.
    */
   public static FixedFrameUnitVector2DBasics newObservableFixedFrameUnitVector2DBasics(ReferenceFrameHolder referenceFrameHolder,
                                                                                        ObjDoubleConsumer<Axis2D> valueChangedListener,
                                                                                        Consumer<Axis2D> valueAccessedListener)
   {
      return newObservableFixedFrameUnitVector2DBasics(valueChangedListener, valueAccessedListener, newFixedFrameUnitVector2DBasics(referenceFrameHolder));
   }

   /**
    * Creates a linked frame unit vector that can be used to observe read and write operations on the
    * source.
    * 
    * @param valueChangedListener  the listener to be notified whenever a coordinate of the unit vector
    *                              has been modified. The corresponding constant {@link Axis2D} will be
    *                              passed to indicate the coordinate that was changed alongside its new
    *                              value. Can be {@code null}.
    * @param valueAccessedListener the listener to be notified whenever a coordinate of the unit vector
    *                              is being accessed. The corresponding constant {@link Axis2D} will be
    *                              passed to indicate the coordinate being accessed. Can be
    *                              {@code null}.
    * @param source                the original unit vector to link and observe. Modifiable via the
    *                              linked unit vector interface.
    * @return the observable frame unit vector.
    */
   public static FixedFrameUnitVector2DBasics newObservableFixedFrameUnitVector2DBasics(ObjDoubleConsumer<Axis2D> valueChangedListener,
                                                                                        Consumer<Axis2D> valueAccessedListener,
                                                                                        FixedFrameUnitVector2DBasics source)
   {
      return newLinkedFixedFrameUnitVector2DBasics(source, newObservableUnitVector2DBasics(valueChangedListener, valueAccessedListener, source));
   }

   /**
    * Creates a new frame unit vector that can be used to observe read and write operations.
    * 
    * @param referenceFrameHolder  the reference frame supplier for the new frame unit vector. Not
    *                              modified.
    * @param valueChangedListener  the listener to be notified whenever a coordinate of the unit vector
    *                              has been modified. The corresponding constant {@link Axis3D} will be
    *                              passed to indicate the coordinate that was changed alongside its new
    *                              value. Can be {@code null}.
    * @param valueAccessedListener the listener to be notified whenever a coordinate of the unit vector
    *                              is being accessed. The corresponding constant {@link Axis3D} will be
    *                              passed to indicate the coordinate being accessed. Can be
    *                              {@code null}.
    * @return the observable frame unit vector.
    */
   public static FixedFrameUnitVector3DBasics newObservableFixedFrameUnitVector3DBasics(ReferenceFrameHolder referenceFrameHolder,
                                                                                        ObjDoubleConsumer<Axis3D> valueChangedListener,
                                                                                        Consumer<Axis3D> valueAccessedListener)
   {
      return newObservableFixedFrameUnitVector3DBasics(valueChangedListener, valueAccessedListener, newFixedFrameUnitVector3DBasics(referenceFrameHolder));
   }

   /**
    * Creates a linked frame unit vector that can be used to observe read and write operations on the
    * source.
    * 
    * @param valueChangedListener  the listener to be notified whenever a coordinate of the unit vector
    *                              has been modified. The corresponding constant {@link Axis3D} will be
    *                              passed to indicate the coordinate that was changed alongside its new
    *                              value. Can be {@code null}.
    * @param valueAccessedListener the listener to be notified whenever a coordinate of the unit vector
    *                              is being accessed. The corresponding constant {@link Axis3D} will be
    *                              passed to indicate the coordinate being accessed. Can be
    *                              {@code null}.
    * @param source                the original unit vector to link and observe. Modifiable via the
    *                              linked unit vector interface.
    * @return the observable frame unit vector.
    */
   public static FixedFrameUnitVector3DBasics newObservableFixedFrameUnitVector3DBasics(ObjDoubleConsumer<Axis3D> valueChangedListener,
                                                                                        Consumer<Axis3D> valueAccessedListener,
                                                                                        FixedFrameUnitVector3DBasics source)
   {
      return newLinkedFixedFrameUnitVector3DBasics(source, newObservableUnitVector3DBasics(valueChangedListener, valueAccessedListener, source));
   }

   /**
    * Creates a new frame rotation matrix that can be used to observe read and write operations.
    * 
    * @param referenceFrameHolder  the reference frame supplier for the new frame rotation matrix. Not
    *                              modified.
    * @param valueChangedListener  the listener to be notified whenever a coordinate of the rotation
    *                              matrix has been modified. The corresponding constant {@link Axis3D}
    *                              will be passed to indicate the coordinate that was changed alongside
    *                              its new value. Can be {@code null}.
    * @param valueAccessedListener the listener to be notified whenever a coordinate of the rotation
    *                              matrix is being accessed. The corresponding constant {@link Axis3D}
    *                              will be passed to indicate the coordinate being accessed. Can be
    *                              {@code null}.
    * @return the observable frame rotation matrix.
    */
   public static FixedFrameRotationMatrixBasics newObservableFixedFrameRotationMatrixBasics(ReferenceFrameHolder referenceFrameHolder,
                                                                                            Runnable valueChangedListener,
                                                                                            BiConsumer<Axis3D, Axis3D> valueAccessedListener)
   {
      return newObservableFixedFrameRotationMatrixBasics(valueChangedListener, valueAccessedListener, newFixedFrameRotationMatrixBasics(referenceFrameHolder));
   }

   /**
    * Creates a linked frame rotation matrix that can be used to observe read and write operations on
    * the source.
    * 
    * @param valueChangedListener  the listener to be notified whenever the rotation matrix has been
    *                              modified. Can be {@code null}.
    * @param valueAccessedListener the listener to be notified whenever a component of the rotation
    *                              matrix is being accessed. The corresponding constants {@link Axis3D}
    *                              will be passed to indicate the row and column respectively of the
    *                              coefficient being accessed. Can be {@code null}.
    * @param source                the original rotation matrix to link and observe. Modifiable via the
    *                              linked rotation matrix interface.
    * @return the observable frame rotation matrix.
    */
   public static FixedFrameRotationMatrixBasics newObservableFixedFrameRotationMatrixBasics(Runnable valueChangedListener,
                                                                                            BiConsumer<Axis3D, Axis3D> valueAccessedListener,
                                                                                            FixedFrameRotationMatrixBasics source)
   {
      return newLinkedFixedFrameRotationMatrixBasics(source, newObservableRotationMatrixBasics(valueChangedListener, valueAccessedListener, source));
   }

   /**
    * Creates a new frame quaternion that can be used to observe read and write operations.
    * 
    * @param referenceFrameHolder  the reference frame supplier for the new frame quaternion. Not
    *                              modified.
    * @param valueChangedListener  the listener to be notified whenever the quaternion has been
    *                              modified. Can be {@code null}.
    * @param valueAccessedListener the listener to be notified whenever a component of the quaternion
    *                              is being accessed. The index of the component being accessed will be
    *                              passed. Can be {@code null}.
    * @return the observable frame quaternion.
    */
   public static FixedFrameQuaternionBasics newObservableFixedFrameQuaternionBasics(ReferenceFrameHolder referenceFrameHolder, Runnable valueChangedListener,
                                                                                    IntConsumer valueAccessedListener)
   {
      return newObservableFixedFrameQuaternionBasics(valueChangedListener, valueAccessedListener, newFixedFrameQuaternionBasics(referenceFrameHolder));
   }

   /**
    * Creates a linked frame quaternion that can be used to observe read and write operations on the
    * source.
    * 
    * @param valueChangedListener  the listener to be notified whenever the quaternion has been
    *                              modified. Can be {@code null}.
    * @param valueAccessedListener the listener to be notified whenever a component of the quaternion
    *                              is being accessed. The index of the component being accessed will be
    *                              passed. Can be {@code null}.
    * @param source                the original quaternion to link and observe. Modifiable via the
    *                              linked quaternion interface.
    * @return the observable frame quaternion.
    */
   public static FixedFrameQuaternionBasics newObservableFixedFrameQuaternionBasics(Runnable valueChangedListener, IntConsumer valueAccessedListener,
                                                                                    FixedFrameQuaternionBasics source)
   {
      return newLinkedFixedFrameQuaternionBasics(source, newObservableQuaternionBasics(valueChangedListener, valueAccessedListener, source));
   }

   /**
    * Creates a new frame bounding box that can be used to observe read and write operations.
    * 
    * @param referenceFrameHolder  the reference frame supplier for the new frame bounding box. Not
    *                              modified.
    * @param valueChangedListener  the listener to be notified whenever a component of the bounding box
    *                              has been modified. The corresponding constants {@link Axis2D} and
    *                              {@link Bound} will be passed to indicate the component that was
    *                              changed alongside its new value. Can be {@code null}.
    * @param valueAccessedListener the listener to be notified whenever a component of the bounding box
    *                              is being accessed. The corresponding constants {@link Axis2D} and
    *                              {@link Bound} will be passed to indicate the component being
    *                              accessed. Can be {@code null}.
    * @return the observable frame bounding box.
    */
   public static FixedFrameBoundingBox2DBasics newObservableFixedFrameBoundingBox2DBasics(ReferenceFrameHolder referenceFrameHolder,
                                                                                          BoundingBoxChangedListener<Axis2D> valueChangedListener,
                                                                                          BiConsumer<Axis2D, Bound> valueAccessedListener)
   {
      return newLinkedFixedFrameBoundingBox2DBasics(referenceFrameHolder, newObservableBoundingBox2DBasics(valueChangedListener, valueAccessedListener));
   }

   /**
    * Creates a linked frame bounding box that can be used to observe read and write operations on the
    * source.
    * 
    * @param valueChangedListener  the listener to be notified whenever a component of the bounding box
    *                              has been modified. The corresponding constants {@link Axis2D} and
    *                              {@link Bound} will be passed to indicate the component that was
    *                              changed alongside its new value. Can be {@code null}.
    * @param valueAccessedListener the listener to be notified whenever a component of the bounding box
    *                              is being accessed. The corresponding constants {@link Axis2D} and
    *                              {@link Bound} will be passed to indicate the component being
    *                              accessed. Can be {@code null}.
    * @param source                the original frame bounding box to link and observe. Modifiable via
    *                              the linked point interface.
    * @return the observable frame bounding box.
    */
   public static FixedFrameBoundingBox2DBasics newObservableFixedFrameBoundingBox2DBasics(BoundingBoxChangedListener<Axis2D> valueChangedListener,
                                                                                          BiConsumer<Axis2D, Bound> valueAccessedListener,
                                                                                          FixedFrameBoundingBox2DBasics source)
   {
      return newLinkedFixedFrameBoundingBox2DBasics(source, newObservableBoundingBox2DBasics(valueChangedListener, valueAccessedListener, source));
   }

   /**
    * Creates a new frame bounding box that can be used to observe read and write operations.
    * 
    * @param referenceFrameHolder  the reference frame supplier for the new frame bounding box. Not
    *                              modified.
    * @param valueChangedListener  the listener to be notified whenever a component of the bounding box
    *                              has been modified. The corresponding constants {@link Axis3D} and
    *                              {@link Bound} will be passed to indicate the component that was
    *                              changed alongside its new value. Can be {@code null}.
    * @param valueAccessedListener the listener to be notified whenever a component of the bounding box
    *                              is being accessed. The corresponding constants {@link Axis3D} and
    *                              {@link Bound} will be passed to indicate the component being
    *                              accessed. Can be {@code null}.
    * @return the observable frame bounding box.
    */
   public static FixedFrameBoundingBox3DBasics newObservableFixedFrameBoundingBox3DBasics(ReferenceFrameHolder referenceFrameHolder,
                                                                                          BoundingBoxChangedListener<Axis3D> valueChangedListener,
                                                                                          BiConsumer<Axis3D, Bound> valueAccessedListener)
   {
      return newLinkedFixedFrameBoundingBox3DBasics(referenceFrameHolder, newObservableBoundingBox3DBasics(valueChangedListener, valueAccessedListener));
   }

   /**
    * Creates a linked frame bounding box that can be used to observe read and write operations on the
    * source.
    * 
    * @param valueChangedListener  the listener to be notified whenever a component of the bounding box
    *                              has been modified. The corresponding constants {@link Axis3D} and
    *                              {@link Bound} will be passed to indicate the component that was
    *                              changed alongside its new value. Can be {@code null}.
    * @param valueAccessedListener the listener to be notified whenever a component of the bounding box
    *                              is being accessed. The corresponding constants {@link Axis3D} and
    *                              {@link Bound} will be passed to indicate the component being
    *                              accessed. Can be {@code null}.
    * @param source                the original frame bounding box to link and observe. Modifiable via
    *                              the linked point interface.
    * @return the observable frame bounding box.
    */
   public static FixedFrameBoundingBox3DBasics newObservableFixedFrameBoundingBox3DBasics(BoundingBoxChangedListener<Axis3D> valueChangedListener,
                                                                                          BiConsumer<Axis3D, Bound> valueAccessedListener,
                                                                                          FixedFrameBoundingBox3DBasics source)
   {
      return newLinkedFixedFrameBoundingBox3DBasics(source, newObservableBoundingBox3DBasics(valueChangedListener, valueAccessedListener, source));
   }
}