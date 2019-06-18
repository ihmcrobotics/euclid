package us.ihmc.euclid.referenceFrame.tools;

import java.util.function.DoubleSupplier;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.euclid.tools.EuclidCoreFactories;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
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
            long bits = 1L;
            bits = EuclidHashCodeTools.addToHashCode(bits, getX());
            bits = EuclidHashCodeTools.addToHashCode(bits, getY());
            return EuclidHashCodeTools.toIntHashCode(bits);
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
            return EuclidCoreIOTools.getTuple2DString(this) + "-" + getReferenceFrame();
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
            long bits = 1L;
            bits = EuclidHashCodeTools.addToHashCode(bits, getX());
            bits = EuclidHashCodeTools.addToHashCode(bits, getY());
            return EuclidHashCodeTools.toIntHashCode(bits);
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
            return EuclidCoreIOTools.getTuple2DString(this) + "-" + getReferenceFrame();
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
            long bits = 1L;
            bits = EuclidHashCodeTools.addToHashCode(bits, getX());
            bits = EuclidHashCodeTools.addToHashCode(bits, getY());
            bits = EuclidHashCodeTools.addToHashCode(bits, getZ());
            return EuclidHashCodeTools.toIntHashCode(bits);
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
            return EuclidCoreIOTools.getTuple3DString(this) + "-" + getReferenceFrame();
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
            long bits = 1L;
            bits = EuclidHashCodeTools.addToHashCode(bits, getX());
            bits = EuclidHashCodeTools.addToHashCode(bits, getY());
            bits = EuclidHashCodeTools.addToHashCode(bits, getZ());
            return EuclidHashCodeTools.toIntHashCode(bits);
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
            return EuclidCoreIOTools.getTuple3DString(this) + "-" + getReferenceFrame();
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
}
