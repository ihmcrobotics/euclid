package us.ihmc.euclid.referenceFrame;

import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;

/**
 * {@code FrameTuple3D} is the base implementation for {@link FramePoint3D} and
 * {@link FrameVector3D}.
 * <p>
 * In addition to representing a {@link Tuple3DBasics}, a {@link ReferenceFrame} is associated to a
 * {@code FrameTuple3D}. This allows, for instance, to enforce, at runtime, that operations on
 * tuples occur in the same coordinate system.
 * </p>
 * <p>
 * Because a {@code FrameTuple3D} extends {@code Tuple3DBasics}, it is compatible with methods only
 * requiring {@code Tuple3DBasics}. However, these methods do NOT assert that the operation occur in
 * the proper coordinate system. Use this feature carefully and always prefer using methods
 * requiring {@code FrameTuple3D}.
 * </p>
 */
public abstract class FrameTuple3D<S extends FrameTuple3D<S, T>, T extends Tuple3DBasics & GeometryObject<T>> extends FrameGeometryObject<S, T>
      implements FrameTuple3DBasics
{
   /** Tuple used to perform the operations. */
   protected final T tuple;

   /**
    * Creates a new frame tuple and initializes its current reference frame and tuple.
    * <p>
    * The given {@code tuple}'s reference is saved internally for performing all the future
    * operations with this {@code FrameTuple3D}.
    * </p>
    * 
    * @param referenceFrame the initial reference frame in which the given tuple is expressed in.
    * @param tuple the tuple that is to be used internally. Reference saved. Will be modified.
    */
   public FrameTuple3D(ReferenceFrame referenceFrame, T tuple)
   {
      super(referenceFrame, tuple);
      this.tuple = getGeometryObject();
   }

   /** {@inheritDoc} */
   @Override
   public final void setX(double x)
   {
      tuple.setX(x);
   }

   /** {@inheritDoc} */
   @Override
   public final void setY(double y)
   {
      tuple.setY(y);
   }

   /** {@inheritDoc} */
   @Override
   public final void setZ(double z)
   {
      tuple.setZ(z);
   }

   @Override
   public void setReferenceFrame(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
   }

   /** {@inheritDoc} */
   @Override
   public final double getX()
   {
      return tuple.getX();
   }

   /** {@inheritDoc} */
   @Override
   public final double getY()
   {
      return tuple.getY();
   }

   /** {@inheritDoc} */
   @Override
   public final double getZ()
   {
      return tuple.getZ();
   }

   /** {@inheritDoc} */
   @Override
   public final void get(Tuple3DBasics tuple3dToPack)
   {
      tuple3dToPack.set(tuple);
   }
}
