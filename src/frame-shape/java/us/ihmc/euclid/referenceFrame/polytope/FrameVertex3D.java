package us.ihmc.euclid.referenceFrame.polytope;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.euclid.referenceFrame.polytope.interfaces.FrameVertex3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameShapeIOTools;
import us.ihmc.euclid.shape.convexPolytope.impl.AbstractVertex3D;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public class FrameVertex3D extends AbstractVertex3D<FrameVertex3D, FrameHalfEdge3D, FrameFace3D> implements FrameVertex3DReadOnly, FixedFramePoint3DBasics
{
   private final ReferenceFrameHolder referenceFrameHolder;

   /**
    * Creates a new vertex and initializes its coordinates.
    *
    * @param x the x-coordinate of this vertex.
    * @param y the y-coordinate of this vertex.
    * @param z the z-coordinate of this vertex.
    */
   public FrameVertex3D(ReferenceFrameHolder referenceFrameHolder, double x, double y, double z)
   {
      super(x, y, z);
      this.referenceFrameHolder = referenceFrameHolder;
   }

   /**
    * Creates a new vertex and initializes its coordinates.
    *
    * @param position the initial position for this vertex. Not modified.
    */
   public FrameVertex3D(ReferenceFrameHolder referenceFrameHolder, Point3DReadOnly position)
   {
      super(position);
      this.referenceFrameHolder = referenceFrameHolder;
   }

   /** {@inheritDoc} */
   @Override
   public FrameHalfEdge3D getEdgeTo(FrameVertex3DReadOnly destination)
   {
      return (FrameHalfEdge3D) FrameVertex3DReadOnly.super.getEdgeTo(destination);
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrameHolder.getReferenceFrame();
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method returns
    * {@link #equals(FrameVertex3DReadOnly)}, it returns {@code false} otherwise.
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      if (object instanceof FrameVertex3DReadOnly)
         return equals((FrameVertex3DReadOnly) object);
      else
         return false;
   }

   /**
    * Calculates and returns a hash code value from the value of each component of this vertex 3D.
    *
    * @return the hash code value for this vertex 3D.
    */
   @Override
   public int hashCode()
   {
      long bits = EuclidHashCodeTools.toLongHashCode(getX(), getY(), getZ());
      bits = EuclidHashCodeTools.addToHashCode(bits, getReferenceFrame());
      return EuclidHashCodeTools.toIntHashCode(bits);
   }

   /**
    * Provides a {@code String} representation of this vertex 3D as follows:
    *
    * <pre>
    * Vertex 3D: (-1.004, -3.379, -0.387 ), number of edges: 3
    *         [(-1.004, -3.379, -0.387 ); ( 1.372, -3.150,  0.556 )]
    *         [(-1.004, -3.379, -0.387 ); (-0.937, -3.539, -0.493 )]
    *         [(-1.004, -3.379, -0.387 ); (-1.046, -3.199, -0.303 )]
    *         worldFrame
    * </pre>
    *
    * @return the {@code String} representing this vertex 3D.
    */
   @Override
   public String toString()
   {
      return EuclidFrameShapeIOTools.getFrameVertex3DString(this);
   }
}
