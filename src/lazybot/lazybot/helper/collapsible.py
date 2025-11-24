import customtkinter as ctk

class CollapsiblePane(ctk.CTkFrame):
    """
    A custom UI widget that acts as a collapsible container.
    It consists of a toggle button and a list of content frames.
    Clicking the button hides or shows the content frames.
    """
    def __init__(self, parent: ctk.CTk | ctk.CTkFrame, contents:list[ctk.CTkFrame] = None, expanded_text: str = "Collapse", collapsed_text: str = "Expand", default_state: bool = True):
        """
        Args:
            parent: The parent widget (Window or Frame).
            contents: A list of CTkFrames to be hidden/shown.
            expanded_text: Text on button when pane is open.
            collapsed_text: Text on button when pane is closed.
            default_state: True to start expanded, False to start collapsed.
        """
        ctk.CTkFrame.__init__(self, parent)
        
        self.parent = parent
        self._contents = contents
        self._expanded_text = expanded_text
        self._collapsed_text = collapsed_text
        
        # Initialize state (inverted because _activate flips it immediately)
        self._is_expanded = not default_state
        
        # Configure grid to stretch horizontally
        self.grid_columnconfigure(0, weight=1)

        # Create the toggle button
        self._button = ctk.CTkButton(self, text=self._collapsed_text, command=self._activate)
        self._button.grid(row=0, column=0, sticky="w")

        # Make the container frame transparent
        self.configure(fg_color="transparent", border_width=0)

        # Apply initial state
        self._activate()
        
    def _activate(self):
        """
        Toggles the visibility of the content frames.
        Called when the button is clicked.
        """
        if(self._is_expanded):
            # Collapse: Hide everything
            self._is_expanded = False
            self._button.configure(text=self._collapsed_text)
            if self._contents is not None:
                for cnt in self._contents:
                    cnt.grid_forget() # Removes widget from layout manager
        else:
            # Expand: Show everything
            self._is_expanded = True
            self._button.configure(text=self._expanded_text)
            if self._contents is not None:
                i = 1
                for cnt in self._contents:
                    # Add padding logic: Top padding for first item, Bottom for last
                    py1 = 0
                    py2 = 20
                    if i == 1:
                        py1 = 20
                    if i == len(self._contents):
                        py2 = 0
                        
                    # Re-grid the content frames
                    cnt.grid(row=i, column=0, padx=0, pady=(py1, py2), sticky="nsew")
                    i += 1
                
    def updateContent(self, new_content:list[ctk.CTkFrame]):
        """
        Dynamically updates the content inside the pane.
        Useful if the UI needs to change based on other selections.
        """
        # Hide old content first
        if self._contents is not None:
            for cnt in self._contents:
                cnt.grid_forget()
                
        self._contents = new_content
        
        # Refresh view (force toggle to ensure correct state)
        # Note: This logic might need adjustment depending on desired behavior (keep open vs reset)
        # Currently, it flips the state, so we might want to force it open or closed explicitly.
        # For now, we just re-run activate to re-grid the new content if it was open.
        
        # Reset state to closed so _activate opens it if we want it open, 
        # or just call _activate to refresh. 
        # Here we assume we want to refresh the current view state.
        state = self._is_expanded
        self._is_expanded = not state # Flip it so _activate flips it back
        self._activate()

