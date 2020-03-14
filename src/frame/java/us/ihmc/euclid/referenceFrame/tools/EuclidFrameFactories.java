package us.ihmc.euclid.referenceFrame.tools;

import java.util.function.DoubleSupplier;

import us.ihmc.euclid.geometry.Orientation2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.euclid.tools.EuclidCoreFactories;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

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
      return newLinkedFramePoint2DReadOnly(EuclidCoreFactories.newLinkedPoint2DReadOnly(scaleSupplier, originalTuple), originalTuple);
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
      return newLinkedFrameVector2DReadOnly(EuclidCoreFactories.newLinkedVector2DReadOnly(scaleSupplier, originalTuple), originalTuple);
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
      return newLinkedFramePoint3DReadOnly(EuclidCoreFactories.newLinkedPoint3DReadOnly(scaleSupplier, originalTuple), originalTuple);
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
      return newLinkedFrameVector3DReadOnly(EuclidCoreFactories.newLinkedVector3DReadOnly(scaleSupplier, originalTuple), originalTuple);
   }

   /**
    * Creates a new point 2D that is a read-only view of the three coordinate suppliers expressed in
    * the reference frame provided by {@code referenceFrameHolder}.
    *
    * @param xSupplier            the x-coordinate supplier.
    * @param ySupplier            the y-coordinate supplier.
    * @param referenceFrameHolder the reference frame supplier. Not modified.
    * @return the new read-only frame point 2D.
    */
   public static FramePoint2DReadOnly newLinkedFramePoint2DReadOnly(DoubleSupplier xSupplier, DoubleSupplier ySupplier,
                                                                    ReferenceFrameHolder referenceFrameHolder)
   {
      return newLinkedFramePoint2DReadOnly(EuclidCoreFactories.newLinkedPoint2DReadOnly(xSupplier, ySupplier), referenceFrameHolder);
   }

   /**
    * Creates a new vector 2D that is a read-only view of the three coordinate suppliers expressed in
    * the reference frame provided by {@code referenceFrameHolder}.
    *
    * @param xSupplier            the x-coordinate supplier.
    * @param ySupplier            the y-coordinate supplier.
    * @param referenceFrameHolder the reference frame supplier. Not modified.
    * @return the new read-only frame vector 2D.
    */
   public static FrameVector2DReadOnly newLinkedFrameVector2DReadOnly(DoubleSupplier xSupplier, DoubleSupplier ySupplier,
                                                                      ReferenceFrameHolder referenceFrameHolder)
   {
      return newLinkedFrameVector2DReadOnly(EuclidCoreFactories.newLinkedVector2DReadOnly(xSupplier, ySupplier), referenceFrameHolder);
   }

   /**
    * Creates a new point 3D that is a read-only view of the three coordinate suppliers expressed in
    * the reference frame provided by {@code referenceFrameHolder}.
    *
    * @param xSupplier            the x-coordinate supplier.
    * @param ySupplier            the y-coordinate supplier.
    * @param zSupplier            the z-coordinate supplier.
    * @param referenceFrameHolder the reference frame supplier. Not modified.
    * @return the new read-only frame point 3D.
    */
   public static FramePoint3DReadOnly newLinkedFramePoint3DReadOnly(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier zSupplier,
                                                                    ReferenceFrameHolder referenceFrameHolder)
   {
      return newLinkedFramePoint3DReadOnly(EuclidCoreFactories.newLinkedPoint3DReadOnly(xSupplier, ySupplier, zSupplier), referenceFrameHolder);
   }

   /**
    * Creates a new vector 3D that is a read-only view of the three coordinate suppliers expressed in
    * the reference frame provided by {@code referenceFrameHolder}.
    *
    * @param xSupplier            the x-coordinate supplier.
    * @param ySupplier            the y-coordinate supplier.
    * @param zSupplier            the z-coordinate supplier.
    * @param referenceFrameHolder the reference frame supplier. Not modified.
    * @return the new read-only frame vector 3D.
    */
   public static FrameVector3DReadOnly newLinkedFrameVector3DReadOnly(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier zSupplier,
                                                                      ReferenceFrameHolder referenceFrameHolder)
   {
      return newLinkedFrameVector3DReadOnly(EuclidCoreFactories.newLinkedVector3DReadOnly(xSupplier, ySupplier, zSupplier), referenceFrameHolder);
   }

   /**
    * Creates a new point 2D that is a read-only view of the point expressed in the reference frame
    * provided by {@code referenceFrameHolder}.
    *
    * @param point                the point to link. Not modified.
    * @param referenceFrameHolder the reference frame supplier. Not modified.
    * @return the new read-only frame point 2D.
    */
   public static FramePoint2DReadOnly newLinkedFramePoint2DReadOnly(Point2DReadOnly point, ReferenceFrameHolder referenceFrameHolder)
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
            return EuclidHashCodeTools.toIntHashCode(point, getReferenceFrame());
         }

         @Override
         public boolean equals(Object object)
         {
            if (object instanceof Point2DReadOnly)
               return FramePoint2DReadOnly.super.equals((Point2DReadOnly) object);
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
    * @param vector               the vector to link. Not modified.
    * @param referenceFrameHolder the reference frame supplier. Not modified.
    * @return the new read-only frame vector 2D.
    */
   public static FrameVector2DReadOnly newLinkedFrameVector2DReadOnly(Vector2DReadOnly vector, ReferenceFrameHolder referenceFrameHolder)
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
            return EuclidHashCodeTools.toIntHashCode(vector, getReferenceFrame());
         }

         @Override
         public boolean equals(Object object)
         {
            if (object instanceof Vector2DReadOnly)
               return FrameVector2DReadOnly.super.equals((Vector2DReadOnly) object);
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
    * @param point                the point to link. Not modified.
    * @param referenceFrameHolder the reference frame supplier. Not modified.
    * @return the new read-only frame point 3D.
    */
   public static FramePoint3DReadOnly newLinkedFramePoint3DReadOnly(Point3DReadOnly point, ReferenceFrameHolder referenceFrameHolder)
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
            return EuclidHashCodeTools.toIntHashCode(point, getReferenceFrame());
         }

         @Override
         public boolean equals(Object object)
         {
            if (object instanceof Point3DReadOnly)
               return FramePoint3DReadOnly.super.equals((Point3DReadOnly) object);
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
    * @param vector               the vector to link. Not modified.
    * @param referenceFrameHolder the reference frame supplier. Not modified.
    * @return the new read-only frame vector 3D.
    */
   public static FrameVector3DReadOnly newLinkedFrameVector3DReadOnly(Vector3DReadOnly vector, ReferenceFrameHolder referenceFrameHolder)
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
            return EuclidHashCodeTools.toIntHashCode(vector, getReferenceFrame());
         }

         @Override
         public boolean equals(Object object)
         {
            if (object instanceof Vector3DReadOnly)
               return FrameVector3DReadOnly.super.equals((Vector3DReadOnly) object);
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
    * Creates a new frame point 2D that is a read-only view of the given {@code originalPoint} negated.
    *
    * @param originalPoint the original point to create linked negative point for. Not modified.
    * @return the negative read-only view of {@code originalPoint}.
    */
   public static FramePoint2DReadOnly newNegativeLinkedFramePoint2D(FramePoint2DReadOnly originalPoint)
   {
      return newLinkedFramePoint2DReadOnly(EuclidCoreFactories.newNegativeLinkedPoint2D(originalPoint), originalPoint);
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
      return newLinkedFrameVector2DReadOnly(EuclidCoreFactories.newNegativeLinkedVector2D(originalVector), originalVector);
   }

   /**
    * Creates a new frame point 3D that is a read-only view of the given {@code originalPoint} negated.
    *
    * @param originalPoint the original point to create linked negative point for. Not modified.
    * @return the negative read-only view of {@code originalPoint}.
    */
   public static FramePoint3DReadOnly newNegativeLinkedFramePoint3D(FramePoint3DReadOnly originalPoint)
   {
      return newLinkedFramePoint3DReadOnly(EuclidCoreFactories.newNegativeLinkedPoint3D(originalPoint), originalPoint);
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
      return newLinkedFrameVector3DReadOnly(EuclidCoreFactories.newNegativeLinkedVector3D(originalVector), originalVector);
   }

   public static FixedFramePoint2DBasics newFixedFramePoint2DBasics(ReferenceFrameHolder referenceFrameHolder)
   {
      return new FixedFramePoint2DBasics()
      {
         private double x, y;

         @Override
         public void setX(double x)
         {
            this.x = x;
         }

         @Override
         public void setY(double y)
         {
            this.y = y;
         }

         @Override
         public double getX()
         {
            return x;
         }

         @Override
         public double getY()
         {
            return y;
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
               return FixedFramePoint2DBasics.super.equals((FramePoint2DReadOnly) object);
            else
               return false;
         }

         @Override
         public int hashCode()
         {
            long bits = EuclidHashCodeTools.addToHashCode(EuclidHashCodeTools.toLongHashCode(x, y), getReferenceFrame());
            return EuclidHashCodeTools.toIntHashCode(bits);
         }

         @Override
         public String toString()
         {
            return EuclidFrameIOTools.getFrameTuple2DString(this);
         }
      };
   }

   public static FixedFrameVector2DBasics newFixedFrameVector2DBasics(ReferenceFrameHolder referenceFrameHolder)
   {
      return new FixedFrameVector2DBasics()
      {
         private double x, y;

         @Override
         public void setX(double x)
         {
            this.x = x;
         }

         @Override
         public void setY(double y)
         {
            this.y = y;
         }

         @Override
         public double getX()
         {
            return x;
         }

         @Override
         public double getY()
         {
            return y;
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
               return FixedFrameVector2DBasics.super.equals((FrameVector2DReadOnly) object);
            else
               return false;
         }

         @Override
         public int hashCode()
         {
            long bits = EuclidHashCodeTools.addToHashCode(EuclidHashCodeTools.toLongHashCode(x, y), getReferenceFrame());
            return EuclidHashCodeTools.toIntHashCode(bits);
         }

         @Override
         public String toString()
         {
            return EuclidFrameIOTools.getFrameTuple2DString(this);
         }
      };
   }

   public static FixedFramePoint3DBasics newFixedFramePoint3DBasics(ReferenceFrameHolder referenceFrameHolder)
   {
      return new FixedFramePoint3DBasics()
      {
         private double x, y, z;

         @Override
         public void setX(double x)
         {
            this.x = x;
         }

         @Override
         public void setY(double y)
         {
            this.y = y;
         }

         @Override
         public void setZ(double z)
         {
            this.z = z;
         }

         @Override
         public double getX()
         {
            return x;
         }

         @Override
         public double getY()
         {
            return y;
         }

         @Override
         public double getZ()
         {
            return z;
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
               return FixedFramePoint3DBasics.super.equals((FramePoint3DReadOnly) object);
            else
               return false;
         }

         @Override
         public int hashCode()
         {
            long bits = EuclidHashCodeTools.addToHashCode(EuclidHashCodeTools.toLongHashCode(x, y, z), getReferenceFrame());
            return EuclidHashCodeTools.toIntHashCode(bits);
         }

         @Override
         public String toString()
         {
            return EuclidFrameIOTools.getFrameTuple3DString(this);
         }
      };
   }

   public static FixedFrameVector3DBasics newFixedFrameVector3DBasics(ReferenceFrameHolder referenceFrameHolder)
   {
      return new FixedFrameVector3DBasics()
      {
         private double x, y, z;

         @Override
         public void setX(double x)
         {
            this.x = x;
         }

         @Override
         public void setY(double y)
         {
            this.y = y;
         }

         @Override
         public void setZ(double z)
         {
            this.z = z;
         }

         @Override
         public double getX()
         {
            return x;
         }

         @Override
         public double getY()
         {
            return y;
         }

         @Override
         public double getZ()
         {
            return z;
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
               return FixedFrameVector3DBasics.super.equals((FrameVector3DReadOnly) object);
            else
               return false;
         }

         @Override
         public int hashCode()
         {
            long bits = EuclidHashCodeTools.addToHashCode(EuclidHashCodeTools.toLongHashCode(x, y, z), getReferenceFrame());
            return EuclidHashCodeTools.toIntHashCode(bits);
         }

         @Override
         public String toString()
         {
            return EuclidFrameIOTools.getFrameTuple3DString(this);
         }
      };
   }

   public static FixedFrameOrientation2DBasics newFixedFrameOrientation2DBasics(ReferenceFrameHolder referenceFrameHolder)
   {
      return new FixedFrameOrientation2DBasics()
      {
         private final Orientation2D orientation2D = new Orientation2D();

         @Override
         public void setYaw(double yaw)
         {
            orientation2D.setYaw(yaw);
         }

         @Override
         public ReferenceFrame getReferenceFrame()
         {
            return referenceFrameHolder.getReferenceFrame();
         }

         @Override
         public double getYaw()
         {
            return orientation2D.getYaw();
         }

         @Override
         public void applyTransform(Transform transform)
         {
            orientation2D.applyTransform(transform);
         }

         @Override
         public void applyInverseTransform(Transform transform)
         {
            orientation2D.applyInverseTransform(transform);
         }

         @Override
         public boolean equals(Object object)
         {
            if (object == this)
               return true;
            else if (object instanceof FrameOrientation2DReadOnly)
               return FixedFrameOrientation2DBasics.super.equals((FrameOrientation2DReadOnly) object);
            else
               return false;
         }

         @Override
         public int hashCode()
         {
            return EuclidHashCodeTools.toIntHashCode(orientation2D, getReferenceFrame());
         }

         @Override
         public String toString()
         {
            return EuclidFrameIOTools.getFrameOrientation2DString(this);
         }
      };
   }

   public static FixedFrameQuaternionBasics newLinkedFixedFrameQuaternionBasics(ReferenceFrameHolder referenceFrameHolder)
   {
      return new FixedFrameQuaternionBasics()
      {
         private double x, y, z, s;

         @Override
         public void setUnsafe(double qx, double qy, double qz, double qs)
         {
            x = qx;
            y = qy;
            z = qz;
            s = qs;
         }

         @Override
         public ReferenceFrame getReferenceFrame()
         {
            return referenceFrameHolder.getReferenceFrame();
         }

         @Override
         public double getX()
         {
            return x;
         }

         @Override
         public double getY()
         {
            return y;
         }

         @Override
         public double getZ()
         {
            return z;
         }

         @Override
         public double getS()
         {
            return s;
         }

         @Override
         public boolean equals(Object object)
         {
            if (object == this)
               return true;
            else if (object instanceof FrameQuaternionReadOnly)
               return FixedFrameQuaternionBasics.super.equals((FrameQuaternionReadOnly) object);
            else
               return false;
         }

         @Override
         public int hashCode()
         {
            long bits = EuclidHashCodeTools.toLongHashCode(x, y, z, s);
            bits = EuclidHashCodeTools.addToHashCode(bits, getReferenceFrame());
            return EuclidHashCodeTools.toIntHashCode(bits);
         }

         @Override
         public String toString()
         {
            return EuclidFrameIOTools.getFrameTuple4DString(this);
         }
      };
   }
}
