package us.ihmc.euclid.shape.convexPolytope;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.shape.convexPolytope.interfaces.Vertex3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;

/**
 * Template data structure that defines a Doubly-connected edge list (DCEL) polytope vertex
 *
 * A DCEL vertex is composed of
 * <li>3D point: A data structure that stores the spatial location of the vertex in 3D space. May /
 * may not have a notion of a {@code ReferenceFrame}
 * <li>Associated edge list: A list of DCEL edges {@code E} that have their origins at this vertex
 *
 * @author Apoorv S
 */
public abstract class Vertex3DBasics implements Vertex3DReadOnly, Point3DBasics
{
   /**
    * List of edges that start at this vertex. May be part of different faces
    */
   private final List<HalfEdge3DBasics> associatedEdges = new ArrayList<>();

   /**
    * Default constructor
    */
   public Vertex3DBasics()
   {
   }

   /**
    * Set the spatial coordinates from another vertex and copy all the edge associations
    */
   public void set(Vertex3DBasics other)
   {
      Point3DBasics.super.set(other);
      clearAssociatedEdgeList();
      addAssociatedEdges(other.getAssociatedEdges());
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public List<HalfEdge3DBasics> getAssociatedEdges()
   {
      return associatedEdges;
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public HalfEdge3DBasics getAssociatedEdge(int index)
   {
      return associatedEdges.get(index);
   }

   /**
    * Method to remove a particular edge from the associated edge list
    *
    * @param edgeToAdd the associated edge that is to be removed. In case the edge specified is not on
    *           the list, no errors are thrown
    *
    */
   public void removeAssociatedEdge(HalfEdge3DBasics edgeToAdd)
   {
      associatedEdges.remove(edgeToAdd);
   }

   /**
    * Remove all edges in the associated edge list
    */
   public void clearAssociatedEdgeList()
   {
      associatedEdges.clear();
   }

   /**
    * Add a {@code List<E>} of DCEL edges to the associated edge list. This invokes the
    * {@code addAssociatedEdge()} method and addition to the list follows the same set of rules
    *
    * @param edgeList a list of DCEL edges that must be added
    */
   public void addAssociatedEdges(List<? extends HalfEdge3DBasics> edgeList)
   {
      for (int i = 0; i < edgeList.size(); i++)
      {
         addAssociatedEdge(edgeList.get(i));
      }
   }

   /**
    * Add a DCEL edge to the associated edge list. In case the edge is already on the associated edge
    * list no action is carried out. The check for whether an edge is already on the list is done by
    * comparing objects. Hence is possible to add a edge that geometrically equals an already existent
    * edge
    *
    * @param edge the DCEL edge to add to the associated edge list
    */
   public void addAssociatedEdge(HalfEdge3DBasics edge)
   {
      if (!isAssociatedWithEdge(edge))
         associatedEdges.add(edge);
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public int getNumberOfAssociatedEdges()
   {
      return getAssociatedEdges().size();
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public String toString()
   {
      return EuclidCoreIOTools.getTuple3DString(this);
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public boolean isAnyFaceMarked()
   {
      boolean isMarked = false;
      for (int i = 0; !isMarked && i < associatedEdges.size(); i++)
      {
         isMarked |= associatedEdges.get(i).getFace().isMarked();
      }
      return isMarked;
   }

   public void round(double epsilon)
   {
      setX(Math.round(getX() / epsilon) * epsilon);
      setY(Math.round(getY() / epsilon) * epsilon);
      setZ(Math.round(getZ() / epsilon) * epsilon);
   }
}
