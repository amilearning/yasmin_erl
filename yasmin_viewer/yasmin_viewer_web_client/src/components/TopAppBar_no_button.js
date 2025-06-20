import * as React from "react";
import AppBar from "@mui/material/AppBar";
import Box from "@mui/material/Box";
import Toolbar from "@mui/material/Toolbar";
import Typography from "@mui/material/Typography";

export default function TopAppBar() {
  return (
    <Box sx={{ flexGrow: 1 }} style={{ width: "100%", height: "6.75vh" }}>
      <AppBar style={{ margin: 0 }}>
        <Toolbar
          sx={{ bgcolor: "black", minHeight: 80, height: 80 }}
          variant="dense"
          disableGutters
        >
          <Typography
            variant="h5"
            component="div"
            style={{ marginLeft: 30 }}
            sx={{
              display: { xs: "none", md: "flex" },
              fontFamily: "monospace",
              fontWeight: 700,
              letterSpacing: ".2rem",
              color: "inherit",
              textDecoration: "none",
            }}
          >
            TUM MIRMI
          </Typography>
        </Toolbar>
      </AppBar>
    </Box>
  );
}