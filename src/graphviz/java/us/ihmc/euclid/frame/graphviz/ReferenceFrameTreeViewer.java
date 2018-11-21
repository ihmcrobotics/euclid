package us.ihmc.euclid.frame.graphviz;

import static guru.nidi.graphviz.model.Factory.*;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.function.Predicate;

import guru.nidi.graphviz.engine.Format;
import guru.nidi.graphviz.engine.Graphviz;
import guru.nidi.graphviz.model.MutableGraph;
import guru.nidi.graphviz.model.MutableNode;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.transform.RigidBodyTransform;

/**
 * Tool used to create a graphical representation of a tree of reference frame as a block diagram
 * saved into a file.
 * 
 * @author Sylvain Bertrand
 */
public class ReferenceFrameTreeViewer
{
   private final ReferenceFrame rootFrame;
   private Format format = Format.SVG;
   private Predicate<ReferenceFrame> filter = frame -> true;
   private final List<LabelProvider> labelProviders = new ArrayList<>();

   /**
    * Creates a new viewer for visualizing the frame subtree starting off the given
    * {@code rootFrameToView}.
    * 
    * @param rootFrameToView the origin of the subtree to visualize.
    */
   public ReferenceFrameTreeViewer(ReferenceFrame rootFrameToView)
   {
      rootFrame = rootFrameToView;
      labelProviders.add(frame -> frame.getName() + ", nameId: " + frame.hashCode());
   }

   /**
    * Registers a custom label provider to display addition information about each frame.
    * 
    * @param labelProvider the custom label provider.
    * @return {@code this} for chaining operations.
    */
   public ReferenceFrameTreeViewer addLabelProvider(LabelProvider labelProvider)
   {
      labelProviders.add(labelProvider);
      return this;
   }

   /**
    * Adds information about the frame's transform to parent.
    * 
    * @return {@code this} for chaining operations.
    */
   public ReferenceFrameTreeViewer addTransformToParentLabel()
   {
      return addLabelProvider(frame -> {
         if (!frame.isRootFrame())
            return "Transform to parent:\n" + getTransformLabel(frame.getTransformToParent());
         else
            return null;
      });
   }

   /**
    * Sets the image format to use.
    * 
    * @param format default value {@code Format.SVG}.
    * @return {@code this} for chaining operations.
    */
   public ReferenceFrameTreeViewer renderingFormat(Format format)
   {
      this.format = format;
      return this;
   }

   /**
    * Adds a filter for selecting the reference frame to be rendered.
    * 
    * @param filter the new filter.
    * @return {@code this} for chaining operations.
    */
   public ReferenceFrameTreeViewer setFilter(Predicate<ReferenceFrame> filter)
   {
      this.filter = filter;
      return this;
   }

   /**
    * Creates the graph and output the block diagram into a file.
    * 
    * @param outputFileName the path to the file to create. The file extension is added internally.
    */
   public void view(String outputFileName)
   {
      MutableGraph graph = mutGraph("ReferenceFrameTreeView").setDirected(true);
      MutableNode rootNode = createFrameNode(rootFrame, graph);

      addChildrenToGraph(rootFrame, rootNode, graph);
      try
      {
         Graphviz.fromGraph(graph).render(format).toFile(new File(outputFileName + "." + format.name().toLowerCase()));
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   private void addChildrenToGraph(ReferenceFrame currentFrame, MutableNode currentNode, MutableGraph graph)
   {
      for (int childIndex = 0; childIndex < currentFrame.getNumberOfChildren(); childIndex++)
      {
         ReferenceFrame child = currentFrame.getChild(childIndex);

         if (filter.test(child))
         {
            MutableNode childNode = createFrameNode(child, graph);
            graph.addLink(currentNode.addLink(childNode));
            addChildrenToGraph(child, childNode, graph);
         }
         else
         {
            addChildrenToGraph(child, currentNode, graph);
         }
      }
   }

   private MutableNode createFrameNode(ReferenceFrame frame, MutableGraph graph)
   {
      String label = labelProviders.get(0).getLabel(frame);

      for (int i = 1; i < labelProviders.size(); i++)
      {
         String additionalLabel = labelProviders.get(i).getLabel(frame);
         if (additionalLabel != null)
            label += "\n" + additionalLabel;
      }

      MutableNode frameNode = mutNode(label);
      graph.add(frameNode);

      return frameNode;
   }

   /**
    * Gets a representative {@code String} of the given values as follows:
    * 
    * <pre>
    * ( 0.123, -0.480,  1.457)
    * </pre>
    * 
    * @param values the values to get the string of.
    * @return the representative string.
    */
   public static String getLabelOf(double... values)
   {
      return EuclidCoreIOTools.getStringOf("(", ")", ", ", values);
   }

   /**
    * Gets a representative {@code String} of the given orientation using a yaw-pitch-roll
    * representation as follows:
    * 
    * <pre>
    * (y,p,r) = ( 0.123, -0.480,  1.457)
    * </pre>
    * 
    * @param orientation the orientation to get the string of. Not modified.
    * @return the representative string.
    */
   public static String getOrientationLabel(Orientation3DReadOnly orientation)
   {
      return "(y,p,r) = " + getLabelOf(orientation.getYaw(), orientation.getPitch(), orientation.getRoll());
   }

   /**
    * Gets a representative {@code String} of the given transform using a yaw-pitch-roll representation
    * for the orientation part as follows:
    * 
    * <pre>
    * (x,y,z) = ( 0.123, -0.480,  1.457)
    * (y,p,r) = ( 0.123, -0.480,  1.457)
    * </pre>
    * 
    * @param transform the transform to get the string of. Not modified.
    * @return the representative string.
    */
   public static String getTransformLabel(RigidBodyTransform transform)
   {
      String translationAsString = transform.getTranslationVector().toString();
      return "(x,y,z) = " + translationAsString + "\n" + getOrientationLabel(transform.getRotationMatrix());
   }

   /**
    * Interface used to implement custom label maker for reference frames.
    * <p>
    * Multiple {@code LabelProvider} can be registered to a viewer to display more information about
    * reference frames.
    * </p>
    * 
    * @author Sylvain Bertrand
    */
   public static interface LabelProvider
   {
      /**
       * Gets the label to display for the given reference frame.
       * 
       * @param frame the frame to make the label from.
       * @return the label to display in the graphical node.
       */
      String getLabel(ReferenceFrame frame);
   }

   /**
    * Generates a basic view of the frame tree starting off {@code rootFrameToView} and saves it in the
    * working directory as <tt>frameView.svg</tt>
    * <p>
    * For generating a custom view, create a new {@link ReferenceFrameTreeViewer}, configure it using
    * {@link #addLabelProvider(LabelProvider)} and {@link #setFilter(Predicate)}, and view it using
    * {@link #view(String)}.
    * </p>
    * 
    * @param rootFrameToView the root of the subtree to view.
    */
   public static void viewSimpleReferenceFrameTree(ReferenceFrame rootFrameToView)
   {
      new ReferenceFrameTreeViewer(rootFrameToView).view("frameView");
   }

   /**
    * Generates a default view of the frame tree starting off {@code rootFrameToView} and saves it in
    * the working directory as <tt>frameView.svg</tt>.
    * <p>
    * For generating a custom view, create a new {@link ReferenceFrameTreeViewer}, configure it using
    * {@link #addLabelProvider(LabelProvider)} and {@link #setFilter(Predicate)}, and view it using
    * {@link #view(String)}.
    * </p>
    * 
    * @param rootFrameToView the root of the subtree to view.
    */
   public static void viewReferenceFrameTree(ReferenceFrame rootFrameToView)
   {
      new ReferenceFrameTreeViewer(rootFrameToView).addTransformToParentLabel().view("frameView");
   }

   /**
    * Example of how to use a reference frame viewer.
    * 
    * @param args the arguments are not used in this example.
    */
   public static void main(String[] args)
   {
      Random random = new Random(435634);
      EuclidFrameRandomTools.nextReferenceFrameTree(random);

      new ReferenceFrameTreeViewer(ReferenceFrame.getWorldFrame()).addTransformToParentLabel().view("ReferenceFrameTreeViewExample");
   }
}
