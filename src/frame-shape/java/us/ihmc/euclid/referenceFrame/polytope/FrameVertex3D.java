package us.ihmc.euclid.referenceFrame.polytope;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.euclid.referenceFrame.polytope.interfaces.FrameHalfEdge3DReadOnly;
import us.ihmc.euclid.referenceFrame.polytope.interfaces.FrameVertex3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameShapeIOTools;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Vertex3DReadOnly;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public class FrameVertex3D implements FrameVertex3DReadOnly, FixedFramePoint3DBasics
{
   private final ReferenceFrameHolder referenceFrameHolder;
   /** The coordinates of this vertex. */
   private double x, y, z;
   /** List of edges that start at this vertex. */
   private final List<FrameHalfEdge3D> associatedEdges = new ArrayList<>();

   /**
    * Creates a new vertex and initializes its coordinates.
    *
    * @param x the x-coordinate of this vertex.
    * @param y the y-coordinate of this vertex.
    * @param z the z-coordinate of this vertex.
    */
   public FrameVertex3D(ReferenceFrameHolder referenceFrameHolder, double x, double y, double z)
   {
      this.referenceFrameHolder = referenceFrameHolder;
      set(x, y, z);
   }

   /**
    * Creates a new vertex and initializes its coordinates.
    *
    * @param position the initial position for this vertex. Not modified.
    */
   public FrameVertex3D(ReferenceFrameHolder referenceFrameHolder, Point3DReadOnly position)
   {
      this.referenceFrameHolder = referenceFrameHolder;
      set(position);
   }

   public List<FrameHalfEdge3D> getAssociatedEdges()
   {
      return associatedEdges;
   }

   @Override
   public FrameHalfEdge3DReadOnly getAssociatedEdge(int index)
   {
      return associatedEdges.get(index);
   }

   /**
    * Removes an edge previously associated to this vertex.
    * <p>
    * In the case the given edge was never associated, nothing changes.
    * </p>
    *
    * @param edgeToRemove the associated edge that is to be removed. Not modified.
    * @return {@code true} if the edge was associated to this vertex, {@code false} otherwise.
    */
   public boolean removeAssociatedEdge(FrameHalfEdge3D edgeToRemove)
   {
      return associatedEdges.remove(edgeToRemove);
   }

   /**
    * Remove all edges previously associated to this vertex.
    */
   public void clearAssociatedEdgeList()
   {
      associatedEdges.clear();
   }

   /**
    * Associates an edge to this vertex, the edge has to start from this vertex.
    * <p>
    * In the case the given edge was already associated, nothing changes.
    * </p>
    *
    * @param edgeToAdd the add to associate to this vertex. Not modified, reference saved.
    * @return {@code true} if the edge was not associated to this vertex, {@code false} otherwise.
    * @throws IllegalArgumentException if {@code edgeToAdd.getOrigin() != this}.
    */
   public boolean addAssociatedEdge(FrameHalfEdge3D edgeToAdd)
   {
      if (!isEdgeAssociated(edgeToAdd))
      {
         if (edgeToAdd.getOrigin() != this)
            throw new IllegalArgumentException("A vertex's associated edges should originate from this same vertex.");
         associatedEdges.add(edgeToAdd);
         return true;
      }
      return false;
   }

   /** {@inheritDoc} */
   @Override
   public FrameHalfEdge3D getEdgeTo(Vertex3DReadOnly destination)
   {
      return (FrameHalfEdge3D) FrameVertex3DReadOnly.super.getEdgeTo(destination);
   }

   /** {@inheritDoc} */
   @Override
   public FrameHalfEdge3D getEdgeTo(FrameVertex3DReadOnly destination)
   {
      return (FrameHalfEdge3D) FrameVertex3DReadOnly.super.getEdgeTo(destination);
   }

   /** {@inheritDoc} */
   @Override
   public int getNumberOfAssociatedEdges()
   {
      return getAssociatedEdges().size();
   }

   /** {@inheritDoc} */
   @Override
   public void setX(double x)
   {
      this.x = x;
   }

   /** {@inheritDoc} */
   @Override
   public void setY(double y)
   {
      this.y = y;
   }

   /** {@inheritDoc} */
   @Override
   public void setZ(double z)
   {
      this.z = z;
   }

   /** {@inheritDoc} */
   @Override
   public double getX()
   {
      return x;
   }

   /** {@inheritDoc} */
   @Override
   public double getY()
   {
      return y;
   }

   /** {@inheritDoc} */
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
      return EuclidHashCodeTools.toIntHashCode(x, y, z);
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
